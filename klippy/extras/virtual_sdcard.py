# Virtual sdcard support (print files directly from a host g-code file)
#
# Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections, os, sys, logging, io

VALID_GCODE_EXTS = ['gcode', 'g', 'gco']

DEFAULT_ERROR_GCODE = """
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
"""

# [DEBUG-motion] v4 bounded host producer timing instrumentation
DEBUG_CHUNK_RING_SIZE = 24
DEBUG_SLOW_RING_SIZE = 16
DEBUG_SLOW_CMD_TIME = 0.020
DEBUG_SLOW_YIELD_TIME = 0.020
DEBUG_CMD_NAME_MAX = 24
DEBUG_DUMP_ENTRIES = 10

def debug_cmd_name(line):
    # Short command name; the g-code line itself is never logged here
    name = line.strip()
    pos = name.find(' ')
    if pos > 0:
        name = name[:pos]
    if len(name) > DEBUG_CMD_NAME_MAX:
        name = name[:DEBUG_CMD_NAME_MAX] + ".."
    return name

class DebugSDChunk:
    # Producer timing for one sdcard read chunk (host side only)
    __slots__ = ('seq', 'start', 'position', 'nbytes', 'read_time',
                 'yield_time', 'line_count', 'cmd_count', 'cmd_time',
                 'peak_cmd_time', 'slow_cmd_time', 'slow_cmd_name',
                 'mutex_pauses', 'mutex_time', 'total_time', 'state')
    def __init__(self, seq, start, position):
        self.seq = seq
        self.start = start
        self.position = position
        self.nbytes = self.line_count = self.cmd_count = 0
        self.mutex_pauses = 0
        self.read_time = self.yield_time = self.cmd_time = 0.
        self.peak_cmd_time = self.slow_cmd_time = 0.
        self.mutex_time = self.total_time = 0.
        self.slow_cmd_name = "-"
        self.state = "active"
    def desc(self):
        return ("seq=%d at=%.6f pos=%d bytes=%d lines=%d read=%.6f"
                " yield=%.6f cmds=%d cmd_time=%.6f peak_cmd=%.6f"
                " slow_cmd=%.6f slow_cmd_name=%s mutex_pauses=%d"
                " mutex_time=%.6f total=%.6f state=%s"
                % (self.seq, self.start, self.position, self.nbytes,
                   self.line_count, self.read_time, self.yield_time,
                   self.cmd_count, self.cmd_time, self.peak_cmd_time,
                   self.slow_cmd_time, self.slow_cmd_name, self.mutex_pauses,
                   self.mutex_time, self.total_time, self.state))

class VirtualSD:
    def __init__(self, config):
        self.printer = config.get_printer()
        # sdcard state
        sd = config.get('path')
        self.sdcard_dirname = os.path.normpath(os.path.expanduser(sd))
        self.current_file = None
        self.file_position = self.file_size = 0
        # Print Stat Tracking
        self.print_stats = self.printer.load_object(config, 'print_stats')
        # Work timer
        self.reactor = self.printer.get_reactor()
        self.must_pause_work = self.cmd_from_sd = False
        self.next_file_position = 0
        self.work_timer = None
        # [DEBUG-motion] v4 bounded producer timing rings
        self.debug_chunk_seq = self.debug_chunk_count = 0
        self.debug_active_chunk = None
        self.debug_chunks = collections.deque([], DEBUG_CHUNK_RING_SIZE)
        self.debug_slow_chunks = collections.deque([], DEBUG_SLOW_RING_SIZE)
        self.debug_slow_cmds = collections.deque([], DEBUG_SLOW_RING_SIZE)
        self.debug_slow_yields = collections.deque([], DEBUG_SLOW_RING_SIZE)
        self.debug_cmd_count = 0
        self.debug_cmd_time = self.debug_read_time = 0.
        self.debug_yield_time = self.debug_worst_yield = 0.
        self.debug_worst_cmd_time = 0.
        self.debug_worst_cmd_name = "-"
        # Error handling
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.on_error_gcode = gcode_macro.load_template(
            config, 'on_error_gcode', DEFAULT_ERROR_GCODE)
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        for cmd in ['M20', 'M21', 'M23', 'M24', 'M25', 'M26', 'M27']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))
        for cmd in ['M28', 'M29', 'M30']:
            self.gcode.register_command(cmd, self.cmd_error)
        self.gcode.register_command(
            "SDCARD_RESET_FILE", self.cmd_SDCARD_RESET_FILE,
            desc=self.cmd_SDCARD_RESET_FILE_help)
        self.gcode.register_command(
            "SDCARD_PRINT_FILE", self.cmd_SDCARD_PRINT_FILE,
            desc=self.cmd_SDCARD_PRINT_FILE_help)
        self.printer.register_event_handler("klippy:analyze_shutdown",
                                            self._handle_analyze_shutdown)
    def _handle_analyze_shutdown(self, msg, details):
        try:
            self._debug_dump_timing()
        except Exception:
            # Diagnostics must never replace the original shutdown reason.
            logging.exception("[DEBUG-motion] Unable to dump sdcard timing")
        if self.work_timer is not None:
            self.must_pause_work = True
            try:
                readpos = max(self.file_position - 1024, 0)
                readcount = self.file_position - readpos
                self.current_file.seek(readpos)
                data = self.current_file.read(readcount + 128)
            except:
                logging.exception("virtual_sdcard shutdown read")
                return
            logging.info("Virtual sdcard (%d): %s\nUpcoming (%d): %s",
                         readpos, repr(data[:readcount]),
                         self.file_position, repr(data[readcount:]))
    # [DEBUG-motion] v4 bounded producer timing
    def _debug_dump_timing(self):
        logging.error(
            "[DEBUG-motion] version=host-v4 sdcard timing chunks=%d cmds=%d"
            " cmd_total=%.6f worst_cmd=%.6f worst_cmd_name=%s read_total=%.6f"
            " yield_total=%.6f worst_yield_over=%.6f pending=%s pos=%d/%d"
            " cmd_from_sd=%s must_pause=%s active=%s",
            self.debug_chunk_count, self.debug_cmd_count, self.debug_cmd_time,
            self.debug_worst_cmd_time, self.debug_worst_cmd_name,
            self.debug_read_time, self.debug_yield_time,
            self.debug_worst_yield, self.debug_active_chunk is not None,
            self.file_position, self.file_size, self.cmd_from_sd,
            self.must_pause_work, self.work_timer is not None)
        chunk = self.debug_active_chunk
        if chunk is not None:
            logging.error("[DEBUG-motion] sdcard chunk pending %s",
                          chunk.desc())
        for c in list(self.debug_slow_chunks)[-DEBUG_DUMP_ENTRIES:]:
            logging.error("[DEBUG-motion] sdcard chunk slow %s", c.desc())
        for c in list(self.debug_chunks)[-DEBUG_DUMP_ENTRIES:]:
            logging.error("[DEBUG-motion] sdcard chunk recent %s", c.desc())
        for at, dur, pos, name, ok in \
                list(self.debug_slow_cmds)[-DEBUG_DUMP_ENTRIES:]:
            logging.error("[DEBUG-motion] sdcard cmd at=%.6f dur=%.6f pos=%d"
                          " name=%s ok=%s", at, dur, pos, name, ok)
        for at, dur, kind, pos in \
                list(self.debug_slow_yields)[-DEBUG_DUMP_ENTRIES:]:
            logging.error("[DEBUG-motion] sdcard yield at=%.6f dur=%.6f"
                          " kind=%s pos=%d", at, dur, kind, pos)
    def _debug_start_chunk(self, t_start, t_end, data):
        # One record per read chunk; retained while the chunk is dispatching
        self.debug_chunk_seq += 1
        chunk = DebugSDChunk(self.debug_chunk_seq, t_start, self.file_position)
        chunk.read_time = t_end - t_start
        self.debug_read_time += chunk.read_time
        self.debug_active_chunk = chunk
        if data is None:
            chunk.nbytes = -1
            self._debug_close_chunk(t_end, "read_error")
        else:
            chunk.nbytes = len(data)
        return chunk
    def _debug_close_chunk(self, t_end, state):
        chunk = self.debug_active_chunk
        if chunk is None:
            return
        self.debug_active_chunk = None
        chunk.state = state
        chunk.total_time = t_end - chunk.start
        self.debug_chunk_count += 1
        self.debug_chunks.append(chunk)
        if (state != "next_read" or chunk.slow_cmd_time > DEBUG_SLOW_CMD_TIME
            or chunk.yield_time > DEBUG_SLOW_YIELD_TIME):
            self.debug_slow_chunks.append(chunk)
    def _debug_note_cmd(self, chunk, line, t_start, t_end, ok):
        dur = t_end - t_start
        self.debug_cmd_count += 1
        self.debug_cmd_time += dur
        if chunk is not None:
            chunk.cmd_count += 1
            chunk.cmd_time += dur
            if dur > chunk.peak_cmd_time:
                chunk.peak_cmd_time = dur
        if dur <= DEBUG_SLOW_CMD_TIME and ok:
            return
        # Slow or failed command - resolve a short name and keep the timing
        name = debug_cmd_name(line)
        if chunk is not None and dur > chunk.slow_cmd_time:
            chunk.slow_cmd_time = dur
            chunk.slow_cmd_name = name
        if dur > self.debug_worst_cmd_time:
            self.debug_worst_cmd_time = dur
            self.debug_worst_cmd_name = name
        self.debug_slow_cmds.append((t_start, dur, self.file_position, name,
                                     ok))
    def _debug_note_yield(self, chunk, t_start, dur, kind, expect):
        self.debug_yield_time += dur
        if chunk is not None:
            if kind == "read":
                chunk.yield_time += dur
            else:
                chunk.mutex_pauses += 1
                chunk.mutex_time += dur
        over = dur - expect
        if over > self.debug_worst_yield:
            self.debug_worst_yield = over
        if over > DEBUG_SLOW_YIELD_TIME:
            self.debug_slow_yields.append((t_start, dur, kind,
                                           self.file_position))
    def stats(self, eventtime):
        if self.work_timer is None:
            return False, ""
        return True, "sd_pos=%d" % (self.file_position,)
    def get_file_list(self, check_subdirs=False):
        if check_subdirs:
            flist = []
            for root, dirs, files in os.walk(
                    self.sdcard_dirname, followlinks=True):
                for name in files:
                    ext = name[name.rfind('.')+1:]
                    if ext not in VALID_GCODE_EXTS:
                        continue
                    full_path = os.path.join(root, name)
                    r_path = full_path[len(self.sdcard_dirname) + 1:]
                    size = os.path.getsize(full_path)
                    flist.append((r_path, size))
            return sorted(flist, key=lambda f: f[0].lower())
        else:
            dname = self.sdcard_dirname
            try:
                filenames = os.listdir(self.sdcard_dirname)
                return [(fname, os.path.getsize(os.path.join(dname, fname)))
                        for fname in sorted(filenames, key=str.lower)
                        if not fname.startswith('.')
                        and os.path.isfile((os.path.join(dname, fname)))]
            except:
                logging.exception("virtual_sdcard get_file_list")
                raise self.gcode.error("Unable to get file list")
    def get_status(self, eventtime):
        return {
            'file_path': self.file_path(),
            'progress': self.progress(),
            'is_active': self.is_active(),
            'file_position': self.file_position,
            'file_size': self.file_size,
        }
    def file_path(self):
        if self.current_file:
            return self.current_file.name
        return None
    def progress(self):
        if self.file_size:
            return float(self.file_position) / self.file_size
        else:
            return 0.
    def is_active(self):
        return self.work_timer is not None
    def do_pause(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            while self.work_timer is not None and not self.cmd_from_sd:
                self.reactor.pause(self.reactor.monotonic() + .001)
    def do_resume(self):
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        self.must_pause_work = False
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.NOW)
    def do_cancel(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
            self.print_stats.note_cancel()
        self.file_position = self.file_size = 0
    # G-Code commands
    def cmd_error(self, gcmd):
        raise gcmd.error("SD write not supported")
    def _reset_file(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
        self.file_position = self.file_size = 0
        self.print_stats.reset()
        self.printer.send_event("virtual_sdcard:reset_file")
    cmd_SDCARD_RESET_FILE_help = "Clears a loaded SD File. Stops the print "\
        "if necessary"
    def cmd_SDCARD_RESET_FILE(self, gcmd):
        if self.cmd_from_sd:
            raise gcmd.error(
                "SDCARD_RESET_FILE cannot be run from the sdcard")
        self._reset_file()
    cmd_SDCARD_PRINT_FILE_help = "Loads a SD file and starts the print.  May "\
        "include files in subdirectories."
    def cmd_SDCARD_PRINT_FILE(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        self._reset_file()
        filename = gcmd.get("FILENAME")
        if filename[0] == '/':
            filename = filename[1:]
        self._load_file(gcmd, filename, check_subdirs=True)
        self.do_resume()
    def cmd_M20(self, gcmd):
        # List SD card
        files = self.get_file_list()
        gcmd.respond_raw("Begin file list")
        for fname, fsize in files:
            gcmd.respond_raw("%s %d" % (fname, fsize))
        gcmd.respond_raw("End file list")
    def cmd_M21(self, gcmd):
        # Initialize SD card
        gcmd.respond_raw("SD card ok")
    def cmd_M23(self, gcmd):
        # Select SD file
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        self._reset_file()
        filename = gcmd.get_raw_command_parameters().strip()
        if filename.startswith('/'):
            filename = filename[1:]
        self._load_file(gcmd, filename)
    def _load_file(self, gcmd, filename, check_subdirs=False):
        files = self.get_file_list(check_subdirs)
        flist = [f[0] for f in files]
        files_by_lower = { fname.lower(): fname for fname, fsize in files }
        fname = filename
        try:
            if fname not in flist:
                fname = files_by_lower[fname.lower()]
            fname = os.path.join(self.sdcard_dirname, fname)
            f = io.open(fname, 'r', newline='')
            f.seek(0, os.SEEK_END)
            fsize = f.tell()
            f.seek(0)
        except:
            logging.exception("virtual_sdcard file open")
            raise gcmd.error("Unable to open file")
        gcmd.respond_raw("File opened:%s Size:%d" % (filename, fsize))
        gcmd.respond_raw("File selected")
        self.current_file = f
        self.file_position = 0
        self.file_size = fsize
        self.print_stats.set_current_file(filename)
    def cmd_M24(self, gcmd):
        # Start/resume SD print
        self.do_resume()
    def cmd_M25(self, gcmd):
        # Pause SD print
        self.do_pause()
    def cmd_M26(self, gcmd):
        # Set SD position
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        pos = gcmd.get_int('S', minval=0)
        self.file_position = pos
    def cmd_M27(self, gcmd):
        # Report SD print status
        if self.current_file is None:
            gcmd.respond_raw("Not SD printing.")
            return
        gcmd.respond_raw("SD printing byte %d/%d"
                         % (self.file_position, self.file_size))
    def get_file_position(self):
        return self.next_file_position
    def set_file_position(self, pos):
        self.next_file_position = pos
    def is_cmd_from_sd(self):
        return self.cmd_from_sd
    # Background work timer
    def work_handler(self, eventtime):
        logging.info("Starting SD card print (position %d)", self.file_position)
        self.reactor.unregister_timer(self.work_timer)
        try:
            self.current_file.seek(self.file_position)
        except:
            logging.exception("virtual_sdcard seek")
            self.work_timer = None
            return self.reactor.NEVER
        self.print_stats.note_start()
        gcode_mutex = self.gcode.get_mutex()
        partial_input = ""
        lines = []
        error_message = None
        # [DEBUG-motion] v4 - bounded producer timing (no per-line logging)
        now = self.reactor.monotonic
        chunk = None
        while not self.must_pause_work:
            if not lines:
                # Read more data
                t_read = now()
                if chunk is not None:
                    self._debug_close_chunk(t_read, "next_read")
                data = None
                try:
                    data = self.current_file.read(8192)
                except:
                    logging.exception("virtual_sdcard read")
                    break
                finally:
                    chunk = self._debug_start_chunk(t_read, now(), data)
                if not data:
                    # End of file
                    self.current_file.close()
                    self.current_file = None
                    logging.info("Finished SD card print")
                    self.gcode.respond_raw("Done printing file")
                    break
                lines = data.split('\n')
                lines[0] = partial_input + lines[0]
                partial_input = lines.pop()
                lines.reverse()
                chunk.line_count = len(lines)
                t_yield = now()
                self.reactor.pause(self.reactor.NOW)
                self._debug_note_yield(chunk, t_yield, now() - t_yield,
                                       "read", 0.)
                continue
            # Pause if any other request is pending in the gcode class
            if gcode_mutex.test():
                t_pause = now()
                self.reactor.pause(t_pause + 0.050)
                self._debug_note_yield(chunk, t_pause, now() - t_pause,
                                       "mutex", 0.050)
                continue
            # Dispatch command
            self.cmd_from_sd = True
            line = lines.pop()
            if sys.version_info.major >= 3:
                next_file_position = self.file_position + len(line.encode()) + 1
            else:
                next_file_position = self.file_position + len(line) + 1
            self.next_file_position = next_file_position
            cmd_ok = False
            t_cmd = t_cmd_end = now()
            try:
                self.gcode.run_script(line)
                t_cmd_end = now()
                cmd_ok = True
            except self.gcode.error as e:
                t_cmd_end = now()
                error_message = str(e)
                try:
                    self.gcode.run_script(self.on_error_gcode.render())
                except:
                    logging.exception("virtual_sdcard on_error")
                break
            except:
                t_cmd_end = now()
                logging.exception("virtual_sdcard dispatch")
                break
            finally:
                self._debug_note_cmd(chunk, line, t_cmd, t_cmd_end, cmd_ok)
            self.cmd_from_sd = False
            self.file_position = self.next_file_position
            # Do we need to skip around?
            if self.next_file_position != next_file_position:
                try:
                    self.current_file.seek(self.file_position)
                except:
                    logging.exception("virtual_sdcard seek")
                    self._debug_close_chunk(now(), "seek_error")
                    self.work_timer = None
                    return self.reactor.NEVER
                lines = []
                partial_input = ""
        if chunk is not None:
            self._debug_close_chunk(now(), "exit")
        logging.info("Exiting SD card print (position %d)", self.file_position)
        self.work_timer = None
        self.cmd_from_sd = False
        if error_message is not None:
            self.print_stats.note_error(error_message)
        elif self.current_file is not None:
            self.print_stats.note_pause()
        else:
            self.print_stats.note_complete()
        return self.reactor.NEVER

def load_config(config):
    return VirtualSD(config)
