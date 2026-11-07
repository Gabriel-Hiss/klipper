"""Python 3.8 determinism probe: threading, queue, select.epoll,
time.monotonic, time.time and multiprocessing under the virtual clock.

Run under LD_PRELOAD=libvclock.so with VCLOCK_SOCK set.
Prints a fixed trace; the harness diffs 20 runs byte for byte.
"""
import multiprocessing
import queue
import select
import threading
import time

_counter = 0
_lock = threading.Lock()


def worker(name, q, n):
    global _counter
    for i in range(n):
        time.sleep(0.005)
        with _lock:
            _counter += 1
            print("W %s %d counter=%d mono=%.6f" % (name, i, _counter, time.monotonic()))
        q.put((name, i))


def child_target(pipe_conn, k):
    pipe_conn.send(k * 2)
    pipe_conn.close()


def main():
    q = queue.Queue()
    threads = [threading.Thread(target=worker, args=(n, q, 4)) for n in ("A", "B")]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    # epoll smoke: pipe readable through epoll
    r, w = __import__("os").pipe()
    import os
    os.write(w, b"x")
    ep = select.epoll()
    ep.register(r, select.EPOLLIN)
    events = ep.poll(1.0)
    print("epoll events=%d" % len(events))
    ep.unregister(r)
    ep.close()
    # multiprocessing fork children must register deterministically
    parent, child = multiprocessing.Pipe()
    proc = multiprocessing.Process(target=child_target, args=(child, 21))
    proc.start()
    print("mp result=%d" % parent.recv())
    proc.join()
    t0 = time.time()
    time.sleep(0.02)
    print("wall=%.6f mono=%.6f done" % (time.time(), time.monotonic()))
    print("main t0=%.6f" % t0)


if __name__ == "__main__":
    main()
