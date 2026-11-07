# Mapa do pipeline de movimento (toolhead, stepper, kinematics, chelper)

Escopo: `klippy/toolhead.py`, `klippy/stepper.py`, `klippy/kinematics/*`, `klippy/chelper/*` (C e `__init__.py`), mais os extras no caminho do movimento: `motion_queuing`, `input_shaper` (+ `shaper_defs`), `gcode_move`, `homing`, `force_move`, `manual_stepper`, `motion_report`, `extruder_stepper`, `stepper_enable`, e o subconjunto de `mathutil` usado por kinematics/shapers.

Método: li os arquivos locais (HEAD `e47b09436a7e` na branch `ender-3-v3-ke`, confirmado em `.git/HEAD` e `.git/refs/heads/ender-3-v3-ke`). Não pude rodar `git` (somente leitura, sem shell). Para achar as mudanças do fork comparei o código local com o upstream `Klipper3d/klipper` master lido por URL. Tudo que não foi comparado com o upstream está marcado como tal. Itens sem prova direta estão marcados `[INFERÊNCIA]`.

Tamanhos são contagens de linhas do arquivo local.

---

## 0. Resumo

1. A máquina alvo usa só `kinematics: cartesian` (`config/ender-3-v3-ke/printer.cfg`: max_velocity 700, max_accel 9000, minimum_cruise_ratio 0.5, square_corner_velocity 12, max_z_velocity 30, max_z_accel 300), 1 extruder com `pressure_advance = 0.030` (smooth_time default 0.040), input shaper `mzv` (X 38.5 Hz, Y 32 Hz, em `config/ender-3-v3-ke/input_shaper.cfg`), `[bltouch]`, `[bed_mesh]`, `[axis_twist_compensation]`, `[exclude_object]`, `[safe_z_home]`, dois MCUs (`[mcu]` serial `/dev/ttyS1` 230400 baud; `[mcu rpi]` host MCU em `/tmp/klipper_host_mcu`). Os outros 13 kinematics são necessários para "parity completa" mas não para a impressora.
2. O núcleo numérico é o C de `chelper/` (trapq, itersolve, kin_*, kin_shaper, kin_extruder, stepcompress). Tudo é IEEE-754 binary64 sem libm exceto `sqrt` (exata) em quase todos, e `atan2/sin/cos` só em polar e rotary_delta. Para paridade bit a bit basta preservar a ordem das operações, não usar FMA e usar casts f64 para inteiro com a mesma semântica de truncamento.
3. O fork **não altera nenhum algoritmo**. Diferenças verificadas contra o upstream: `BUFFER_TIME_HIGH` 1.0 para 2.0 em `toolhead.py`; `BGFLUSH_SG_LOW_TIME` 0.450 para 1.000 e `BGFLUSH_SG_HIGH_TIME` 0.700 para 1.250 em `motion_queuing.py`; e instrumentação `[DEBUG-motion]` em `motion_queuing.py`, `itersolve.c`, `stepcompress.c`, `serialqueue.c`, `reactor.py`, `mcu.py`, `klippy.py`, `virtual_sdcard.py` (as 4 últimas fora do meu recorte).
4. Os testes `test/klippy/*.test` só checam código de saída de `klippy.py -i gcode -o saida -d dict`; nenhum compara saída. Parity precisa de oráculos novos (seção 8).
5. O C pode ficar atrás de FFI numa porta incremental, mas só em blocos coesos: {trapq, itersolve, kin_*, shaper, extruder} andam juntos (lista intrusiva `struct move`), e {msgblock, stepcompress, steppersync, serialqueue, trdispatch} andam juntos (lista intrusiva `struct queue_message`). Detalhes na seção 7.

---

## 1. Fluxo de ponta a ponta (G1 até o fio)

```
gcode G1  -> gcode_move.cmd_G1 -> move_with_transform (bed_mesh/skew/... ou toolhead.move)
toolhead.move(newpos, speed)
  Move(...)                         # planejamento por movimento, em Python
  kin.check_move(move)              # limites, z_ratio, homed
  extra_axes[i].check_move(...)     # extruder/manual_stepper
  lookahead.add_move(move)          # calc_junction com o anterior; retorna want_flush
  se want_flush: _process_lookahead(lazy=True)
_process_lookahead
  lookahead.flush(lazy)             # passe reverso + passe direto, set_junction
  estado "NeedPrime"/"Priming" -> "" ; _calc_print_time()
  trapq_append (C) para cada Move cinemático; ea.process_move (extruder grava no trapq do extruder)
  motion_queuing.note_mcu_movequeue_activity(next_move_time)   # chuta flush_timer
motion_queuing._flush_handler (timer do reactor)
  _advance_flush_time(flush_time, step_gen_time)
    callbacks: toolhead._handle_step_flush (primeiro), stepper._check_active, output_pin, pwm_tool...
    steppersyncmgr_gen_steps (C, uma thread pthread por stepper):
       itersolve_generate_steps -> calc_position_cb -> stepcompress_append
       stepcompress_flush (queue_flush -> compress_bisect_add -> add_move cria queue_step)
       stepcompress_history_expire
       depois, por MCU: steppersync_flush ordena por req_clock e chama serialqueue_send_batch
    trapq_finalize_moves por trapq
serialqueue (thread C com poll): envia blocos com CRC16, retransmite, mantém janela
```

Homing/probing usa "drip": `toolhead.drip_move` -> `motion_queuing.drip_update_time` flushando em segmentos de 50 ms até `drip_completion`.

---

## 2. Fronteira Python para C (cffi modo ABI, `dlopen`)

`chelper/__init__.py` compila `c_helper.so` com `gcc -Wall -g -O2 -shared -fPIC -flto -fwhole-program -fno-use-linker-plugin` (mais `-mfpmath=sse -msse2` se o gcc aceitar), só se algum fonte for mais novo que o `.so`. Declara os protótipos à mão em `defs_*` e abre com `ffi.dlopen`. Não há verificação de layout: `struct pull_move`, `struct pull_history_steps`, `struct pull_queue_message` precisam bater com o C manualmente.

Cada função só é visível se tiver `__visible` (`-fwhole-program`). Funções internas como `stepcompress_append`, `itersolve_generate_steps`, `serialqueue_send_batch`, `trapq_check_sentinels` NÃO são chamáveis do Python.

| Módulo C | Funções em cdef | Quem chama |
|---|---|---|
| pyhelper | `set_python_logging_callback`, `get_monotonic` (CLOCK_MONOTONIC_RAW), `set_thread_name` | `chelper.get_ffi`; `reactor.monotonic = get_monotonic`; `serialhdl._bg_thread` |
| serialqueue | `serialqueue_alloc/exit/free`, `serialqueue_alloc_commandqueue/free_commandqueue`, `serialqueue_send`, `serialqueue_pull`, `set_wire_frequency`, `set_receive_window`, `set_clock_est`, `get_stats`, `extract_old` | `serialhdl.py` (fora do meu recorte) |
| stepcompress | `stepcompress_fill`, `set_invert_sdir`, `reset`, `set_last_position`, `find_past_position`, `extract_old` | `stepper.py` |
| steppersync | `syncemitter_get_stepcompress`, `syncemitter_set_stepper_kinematics`, `syncemitter_get_stepper_kinematics`, `syncemitter_queue_msg`, `steppersync_alloc_syncemitter`, `steppersync_setup_movequeue`, `steppersync_set_time`, `steppersyncmgr_alloc/free`, `steppersyncmgr_alloc_steppersync`, `steppersyncmgr_gen_steps` | `motion_queuing.py`, `stepper.py`, `pwm_tool.py` |
| itersolve | `itersolve_check_active`, `itersolve_is_active_axis`, `itersolve_set_trapq`, `itersolve_get_trapq`, `itersolve_calc_position_from_coord`, `itersolve_set_position`, `itersolve_get_commanded_pos`, `itersolve_get_gen_steps_pre_active/post_active` | `stepper.py`, `motion_queuing.py`, `kinematics/rotary_delta.py` |
| trapq | `trapq_alloc/free`, `trapq_append`, `trapq_finalize_moves`, `trapq_set_position`, `trapq_extract_old` | `toolhead.py`, `motion_queuing.py`, `extruder.py`, `force_move.py`, `manual_stepper.py`, `motion_report.py` |
| trdispatch | `trdispatch_alloc`, `trdispatch_mcu_alloc`, `trdispatch_mcu_setup`, `trdispatch_start/stop` | `mcu.py` (MCU_trsync / MCU_endstop) |
| kin_* | `cartesian_stepper_alloc(char)`, `corexy_stepper_alloc(char)`, `corexz_stepper_alloc(char)`, `delta_stepper_alloc(arm2,tx,ty)`, `deltesian_stepper_alloc(arm2,arm_x)`, `polar_stepper_alloc(char)`, `rotary_delta_stepper_alloc(5 doubles)`, `winch_stepper_alloc(3)`, `generic_cartesian_stepper_alloc(3)` + `_set_coeffs`, `extruder_stepper_alloc/free`, `extruder_set_pressure_advance`, `input_shaper_alloc/set_sk/set_shaper_params/update_sk`, `dual_carriage_alloc/set_sk/set_transform` | `stepper.setup_itersolve` (por nome via `getattr(ffi_lib, alloc_func)`), `kinematics/extruder.py`, `input_shaper.py`, `idex_modes.py`, `kinematic_stepper.py` |
| libc | `free` | `ffi.gc(sk, ffi_lib.free)` |

Estruturas compartilhadas por layout (cdef): `pull_move{print_time,move_t,start_v,accel,start_x,y,z,x_r,y_r,z_r}` (10 doubles), `pull_history_steps{u64 first_clock,last_clock; i64 start_position; int step_count,interval,add}`, `pull_queue_message{u8 msg[64]; int len; double sent_time,receive_time; u64 notify_id}`. Python lê campos diretamente (`data[i].first_clock`, etc.).

Callback C para Python: `set_python_logging_callback(void(*)(const char*))`. `errorf()` (pyhelper.c, buffer de 512 bytes) é chamada de qualquer thread C (step threads, serialqueue) e entra no Python (adquire a GIL) para `logging.error`. Os `[DEBUG-motion]` do C passam por aqui.

Ownership (hoje feito por `ffi.gc` e coleta do Python):
- `trapq`: `ffi.gc(trapq_alloc(), trapq_free)`, guardado em `motion_queuing.trapqs`.
- `steppersyncmgr`: `ffi.gc(..., steppersyncmgr_free)`; ele libera steppersync, syncemitter e threads.
- `stepper_kinematics`: `ffi.gc(alloc(), ffi_lib.free)` (extruder usa `extruder_stepper_free`). Ponteiros cruzados sem contagem de referência: `input_shaper.orig_sk`, `dual_carriage.orig_sk`, `syncemitter.sk`, `sk.tq`. Python mantém os originais vivos em listas (`orig_stepper_kinematics`, `DualCarriagesRail.sks`).
- `stepcompress` pertence ao syncemitter (alocado com `alloc_stepcompress=True`). `pwm_tool` usa `alloc_stepcompress=False` (só fila de mensagens).

Concorrência na fronteira: as chamadas cffi liberam a GIL. Durante `steppersyncmgr_gen_steps` a thread principal fica bloqueada no C esperando as threads por stepper; por isso não há mutação concorrente de `sk`/`trapq`. Esta propriedade é o que permite a um port em Rust modelar a geração como região paralela com escopo.

---

## 3. Unidades C (`klippy/chelper/`)

Todas as unidades C são Linux apenas (`linux/can.h`, `sys/prctl.h`, `poll`, `termios`, `pthread`). Não compilam no Windows host.

Formato: **nome (linhas)** | papel | dependências | contrato externo | traços difíceis | testes | fork | dificuldade.

### 3.1 `__init__.py` (351)
- Papel: build do `.so` via gcc em tempo de execução, cdef, `get_ffi()`, `run_hub_ctrl` (compila `lib/hub-ctrl` e chama `sudo hub-ctrl -h 0 -P 2 -p N`, usado pelo `output_pin`/`hub` extras; fora do meu recorte funcional).
- Deps: `cffi`, `gcc` no host, `os.system`.
- Contrato: log `"Building C code module c_helper.so"`, erro `"Unable to build C code module (error=%s)"`. Arquivo `klippy/chelper/c_helper.so` e `_temp_c_helper.so`. Modo `__main__` compila.
- Traços: build no dispositivo (precisa gcc na KE se o `.so` não for pré-compilado), flags de FP (SSE) decididas por teste do gcc.
- Testes: todos (importado por toda config).
- Fork: nenhuma diferença encontrada (não comparei linha a linha).
- Dificuldade: baixa (desaparece), mas a lista `defs_*` é a especificação da ABI a manter durante a transição.

### 3.2 `trapq.c` (273) + `trapq.h` (~62)
- Papel: fila trapezoidal. `struct move{print_time, move_t, start_v, half_accel, start_pos{x,y,z}, axes_r{x,y,z}, node}`; `struct trapq{moves, history}` (listas intrusivas). `trapq_append` quebra um movimento em até 3 segmentos (accel, cruise, decel) com `half_accel=.5*accel`, `start_pos` encadeado via `move_get_coord(m, t)`. `trapq_add_move` insere "null move" de preenchimento (limitado a `MAX_NULL_MOVE=1.0` s quando `prev->print_time <= 0`). Sentinelas: head (`print_time=-1`), tail (`print_time=NEVER_TIME 9999999999999999.9`, atualizado por `trapq_check_sentinels` para fim do último movimento e `start_pos` final). `trapq_finalize_moves(print_time, clear_history_time)` move expirados para `history` (só se `start_v||half_accel`, o resto é liberado) e libera histórico antigo. `trapq_set_position` finaliza tudo, poda histórico interrompido e insere marcador. `trapq_extract_old` serve motion_report.
- Deps: `list.h`, `compiler.h`. Python: nenhuma além do cffi.
- Contrato: ver tabela 2; semântica de `axes_r`: no trapq do extruder `axes_r.x=1` (posição do extruder em `start_pos.x`) e `axes_r.y` carrega a flag de pressure advance (0 ou 1) (ver extruder.process_move).
- Traços: aritmética idêntica nos dois lados (`move_get_distance = (start_v + half_accel*t)*t`); sentinela tail com `print_time=0` como flag "precisa atualizar"; histórico em ordem mais novo primeiro (`list_add_head`).
- Testes: indiretos em todos.
- Fork: sem marcador; não comparado linha a linha.
- Dificuldade: média (estruturas de dados fáceis, mas vizinhos `list_prev_entry/next_entry` usados por shaper e PA assumem sentinelas e preenchimento sem lacunas).

### 3.3 `itersolve.c` (355) + `itersolve.h`
- Papel: dado `stepper_kinematics{step_dist, commanded_pos, last_flush_time, last_move_time, tq, active_flags, gen_steps_pre_active/post_active, calc_position_cb, post_cb}` gera os tempos de passo: para cada trecho de movimento resolve `calc_position_cb(sk, m, t) == target` por método da secante com colchete (`low_time/high_time`), bisseção, detecção de inversão de direção, anti-oscilação, tolerâncias `1e-9` e `1e-8`+`2*half_step`, `SEEK_TIME_RESET=1e-4`. Chama `stepcompress_append(sc, sdir, m->print_time, guess.time)` e `stepcompress_commit`. `itersolve_generate_steps` percorre o trapq pulando movimentos inativos (`check_active` via `active_flags` e `axes_r != 0`) e gera passos extras antes/depois da atividade (`gen_steps_pre_active/post_active`, usados por shaper e pressure advance).
- Deps: trapq, stepcompress (chamada direta C, não via cdef).
- Contrato Python: `itersolve_check_active(sk, flush_time)` retorna o `print_time` do primeiro movimento ativo ou 0 (dispara `add_active_callback`/enable do motor); `itersolve_calc_position_from_coord(sk,x,y,z)` (monta um `struct move` temporário com `move_t=1000`, avalia em `t=500`); `itersolve_set_position`; getters.
- Traços: ponto flutuante sensível que chega ao MCU (seção 5); estado de máquina com muitas variáveis; UB potencial se o movimento anterior à cabeça acabar.
- Testes: todos que geram passos.
- Fork: **instrumentação**. `itersolve_debug_dump` (linhas 28 a 82) e `debug_reported`; o valor de retorno de `stepcompress_commit` passou a ser capturado só para o dump, o comportamento original (ignorar o retorno) foi preservado (comentário no código). Resto do laço idêntico ao upstream (comparei as linhas 15 a 140 do upstream).
- Dificuldade: alta (paridade bit a bit; caminho único de erro "Internal error in stepcompress").

### 3.4 `stepcompress.c` (782) + `stepcompress.h`
- Papel: comprime a série de tempos de passo (relógios MCU) em comandos `queue_step interval count add` (`compress_bisect_add`: bisseção sobre `add` com limites min/max por ponto, `QUADRATIC_DEV 11`, preferência por `add=0` se `zerocount + zerocount/16 >= bestcount`). Filtro step+dir+step (`SDS_FILTER_TIME .000750`, rollback do último passo). Histórico de comandos (`history_steps`) para `find_past_position` e `extract_old`. Mensagens `set_next_step_dir` e `queue_step` são codificadas em VLQ por `message_alloc_and_encode` e colocadas em `sc->msg_queue` (lista do syncemitter). `CLOCK_DIFF_MAX (3<<28)`, `queue_step` isolado para passos distantes.
- Deps: msgblock.c (`message_alloc_and_encode`), serialqueue.h (`struct queue_message`), pyhelper (`errorf`).
- Contrato: ver tabela 2. Mensagens MCU geradas: `queue_step oid=%c interval=%u count=%hu add=%hi`, `set_next_step_dir oid=%c dir=%c` (dir XOR `invert_sdir`). Retorno `ERROR_RET -989898989` leva a "Internal error in stepcompress" (shutdown).
- Traços: aritmética de inteiros de 32 bits com wrap esperado (`uint32 point = clock32 - lsc`; produtos `nextmininterval*nextcount` e `add*nextaddfactor` em `int32_t` podem estourar: UB em C, na prática wrap); `idiv_up/idiv_down` com semântica especial para negativos; `(uint64_t)rel_sc` com double possivelmente negativo (UB; comportamento x86-64 é wrap por `cvttsd2si`; em MIPS32 o libgcc pode se comportar diferente `[INFERÊNCIA]`).
- Testes: todos.
- Fork: **instrumentação no caminho quente**. `stepcompress_append` agora grava um `struct debug_append` (~100 bytes) por passo em anel de 24 entradas, `debug_record_event` em `set_time`, `add_move`, `set_next_step_dir`, `reset` (anel de 8); `debug_dump` (uma vez por objeto, até ~80 linhas `[DEBUG-motion] version=motion-v3 ...`) em `check_line` e `queue_flush`. O algoritmo e as constantes são idênticos ao upstream (comparei `compress_bisect_add`, `add_move`, `queue_flush`, `queue_append*`, `stepcompress_append`). O fork aumentou `sizeof(struct stepcompress)` em ~3 KB por stepper e escreve ~100 bytes por passo.
- Dificuldade: alta (precisa de wrapping explícito e fuzz diferencial; é o ponto de maior risco para "step times bit-idênticos").

### 3.5 `steppersync.c` (436) + `steppersync.h`
- Papel: `syncemitter` (1 por stepper/PWM, com `stepcompress` opcional, `stepper_kinematics`, fila de mensagens e **uma thread pthread nomeada** com mutex/cond), `steppersync` (1 por MCU: `clock_estimate`, fila de relógios de "move queue" do MCU como heap mínima, lista de syncemitters), `steppersyncmgr` (lista de steppersync). `steppersyncmgr_gen_steps(flush_time, gen_steps_time, clear_history_time)`: (1) `trapq_check_sentinels` de cada trapq; (2) converte tempos para relógio por MCU (`clock_from_time`, arredonda com +.5) e dispara as threads; (3) espera todas; (4) se nenhuma falhou, `steppersync_flush` por MCU: escolhe a mensagem de menor `req_clock` entre as cabeças das filas (empate: primeiro da lista, `<` estrito), usa `min_clock` sobrecarregado como "usa slot da move queue" (`heap_replace`), e envia o lote por `serialqueue_send_batch`. Retorna o `syncemitter*` que falhou (Python mapeia para nome).
- Deps: itersolve, stepcompress, serialqueue, trapq, pyhelper.
- Contrato: `syncemitter_queue_msg(se, req_clock, data, len)` (usado por `stepper.note_homing_end` para `reset_step_clock oid=%c clock=%u` e por `pwm_tool`), `steppersync_set_time(ss, offset, freq)` chamado em `setup_mcu_movequeue` e a cada `motion_queuing.stats`.
- Traços: threads, pthread cond; resultado determinístico independente do agendamento das threads (as mensagens só entram na fila serial após todas as threads terminarem).
- Testes: todos.
- Fork: nenhum marcador; não comparei linha a linha.
- Dificuldade: média.

### 3.6 `serialqueue.c` (1430) + `serialqueue.h`
- Papel: protocolo de blocos do MCU sobre UART/CAN/arquivo: thread de fundo com `pollreactor` (2 fds, 2 timers), fila `upcoming`/`ready` por `command_queue`, prioridade por `req_clock`, liberação por `min_clock` contra o relógio estimado do MCU (`MIN_REQTIME_DELTA 0.100`, `MIN_BACKGROUND_DELTA 0.005`), janela `MAX_PENDING_BLOCKS 12` e `receive_window`, retransmissão RFC6298 (`MIN_RTO .025`, `MAX_RTO 5.0`), fast readers (trsync), fila de notificações (`send_wait_ack`), recepção e CRC16, pipe de wake. Modo `f` (debug file) escreve sem ler.
- Deps: msgblock, pollreactor, pyhelper, list.h.
- Contrato MCU: formato de bloco `[len][seq|0x10][payload][crc16 hi][crc16 lo][0x7E]`; `serialqueue_get_stats` produz string `bytes_write=... send_seq=... srtt=...` (parseada por `statistics`/`graphstats.py` `[INFERÊNCIA: não verifiquei os parsers]`).
- Traços: threads, `poll`, `termios` (`tcflush`), `can_frame`, temporização real; `get_monotonic` é `CLOCK_MONOTONIC_RAW`.
- Testes: só no modo arquivo (`-o`); caminho UART/CAN não é testado.
- Fork: **instrumentação** `sq_debug_*` (anéis de 64 + 16 eventos, banner `[DEBUG-motion] version=motion-v4 serial=... init`, dump `sq_debug_dump` chamado de dentro de `serialqueue_extract_old(sentq=1)`). `sq_debug_note` roda dentro do lock em `check_send_command`, `build_and_send_command`, `do_write`, `retransmit_event`. Sem mudança de política; não comparei contra o upstream linha a linha.
- Dificuldade: alta.

### 3.7 `msgblock.c` (224) + `msgblock.h`
- Papel: `msgblock_crc16_ccitt`, `msgblock_check` (resincroniza em 0x7E), VLQ `encode_int/parse_int`, `msgblock_decode`, alocação de `queue_message`, `clock_from_clock32`, `clock_to_time`, `clock_from_time` (`(int64_t)((t - conv_time)*freq + .5) + conv_clock`), `clock_fill`.
- Contrato: codificação de comandos para o MCU (mesma do `msgproto.py` em Python, fora do meu recorte).
- Dificuldade: baixa.

### 3.8 `pollreactor.c` (179) + `.h`
- Papel: loop `poll()` com timers; `PR_NOW=0.`, `PR_NEVER=9999999999999999.`; timeout `ceil(dt*1000)` limitado a [1,1000] ms.
- Dificuldade: baixa.

### 3.9 `pyhelper.c` (102) + `.h`
- Papel: `get_monotonic`, `errorf`, `report_errno`, `dump_string`, `set_thread_name` (`prctl PR_SET_NAME`), `fill_time`.
- Fork: nenhum marcador.
- Dificuldade: baixa. `errorf` precisa de um sink thread-safe (hoje reentra no Python).

### 3.10 `trdispatch.c` (228)
- Papel: despacho de `trsync` (homing multi-MCU) em baixa latência: fast reader de `trsync_state`, propaga `trsync_trigger` e estende `trsync_set_timeout` em todos os MCUs.
- Mensagens: `trsync_trigger oid=%c reason=%c`, `trsync_set_timeout oid=%c clock=%u`, entrada `trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u`.
- Traços: callback chamado da thread serial com `fast_reader_dispatch_lock`; usa `eventtime` real.
- Testes: só o caminho de arquivo (`is_fileoutput` curto-circuita).
- Dificuldade: média.

### 3.11 `list.h` (131), `compiler.h` (46)
- Lista intrusiva estilo kernel; macros `likely/unlikely/__visible/container_of/DIV_ROUND_UP`. Dificuldade: baixa (substituir por Vec/VecDeque/arena).

### 3.12 Solvers cinemáticos `kin_*.c` (cada um define `calc_position_cb` e `active_flags`)

| Arquivo (linhas) | Posição do stepper | libm | Observações | Dificuldade |
|---|---|---|---|---|
| `kin_cartesian.c` (52) | `x`, `y` ou `z` de `move_get_coord` | nenhuma | `cartesian_stepper_alloc(char axis)` | baixa |
| `kin_corexy.c` (40) | `x+y` (`'+'`) ou `x-y` | nenhuma | `AF_X|AF_Y` | baixa |
| `kin_corexz.c` (40) | `x+z` ou `x-z` | nenhuma | `AF_X|AF_Z` | baixa |
| `kin_delta.c` (41) | `sqrt(arm2 - dx*dx - dy*dy) + c.z`, `dx=tower_x-c.x` | sqrt | ordem `(arm2 - dx*dx) - dy*dy` | baixa |
| `kin_deltesian.c` (41) | `sqrt(arm2 - dx*dx) + c.z`, `dx=c.x-arm_x` | sqrt | `AF_X|AF_Z` | baixa |
| `kin_polar.c` (59) | raio `sqrt(x*x+y*y)`; ângulo `atan2(y,x)` com ajuste de ±2π relativo a `sk->commanded_pos` e `post_cb` que normaliza `commanded_pos` | sqrt, **atan2** | estado em `commanded_pos`; `M_PI`; `// XXX handle x==y==0` | média |
| `kin_rotary_delta.c` (73) | ângulo do braço por interseção de circunferências | sqrt, **atan2**, **cos/sin** (só no alloc) | `rotary_two_arm_calc` com `1./dy` | média |
| `kin_winch.c` (42) | `sqrt(dx²+dy²+dz²)` até a âncora | sqrt | | baixa |
| `kin_generic.c` (51) | `a.x*c.x + a.y*c.y + a.z*c.z`; `set_coeffs` recalcula `active_flags` (coeficiente != 0) | nenhuma | ordem `(ax*cx + ay*cy) + az*cz` | baixa |
| `kin_idex.c` (82) | envelope sobre outro `sk`: aplica `scale/offset` em x e y e chama `orig_sk->calc_position_cb(orig, &dc->m, 500.)` com `dc->m.start_pos` | nenhuma | `dual_carriage_set_transform` ajusta `active_flags` (scale 0 desativa o eixo); `orig_sk->commanded_pos` não é atualizado | média |
| `kin_extruder.c` (200) | posição do extruder com pressure advance e suavização | nenhuma | ver abaixo | alta |
| `kin_shaper.c` (294) | convolução do input shaper | nenhuma (inclui math.h) | ver abaixo | alta |

`kin_extruder.c`: sem PA retorna `start_pos.x + move_get_distance(m, t)`. Com PA: média ponderada triangular sobre `[t-hst, t+hst]` (`hst=smooth_time/2`) de `pos(t) + pa*vel(t)`, via integrais definidas `extruder_integrate` e `extruder_integrate_time`, atravessando movimentos vizinhos com `list_prev_entry/next_entry`. PA só se aplica em movimentos com `axes_r.y != 0` (flag vinda do Python). Valor de PA vigente: lista `pa_params` ordenada por `active_print_time` (busca do fim para trás). `extruder_set_pressure_advance(print_time, pa, smooth_time)` limpa entradas antigas com base em `last_flush_time`, define `gen_steps_pre_active = post_active = hst`, `inv_half_smooth_time2 = 1./(hst*hst)`, e só empilha nova entrada se o PA mudou. Constantes: `(1./3.)`, `.25*half_accel`.

`kin_shaper.c`: `init_shaper(n, a[], t[])` normaliza `a` por `1./sum_a`, inverte a ordem (`pulses[n-i-1]`, `t=-t[i]`), `shift_pulses` subtrai `ts = sum(a*t)` para que o shaper seja identidade em velocidade constante e acha `p_ind` (primeiro `t>0`). `calc_position` varre pulsos para trás (do `p_ind-1` até 0) e para frente acumulando `t += pulses[i].t; ...; t -= pulses[i].t;` (a sequência soma e subtrai cria erro de arredondamento que faz parte do resultado), atravessando movimentos vizinhos. Quatro variantes de `calc_position_cb` (x, y, z, xyz). Usa um `struct move m` privado cujo `start_pos` é a posição suavizada e chama `orig_sk->calc_position_cb(orig, &is->m, 500.)`. `shaper_note_generation_time` calcula `gen_steps_pre_active/post_active` pelos pulsos extremos. Máximo de 10 pulsos por eixo. Retorna -1 em tempos não crescentes.

Fork nos `kin_*`: nenhum marcador; comparei só `kin_shaper.c` linhas 30 a 130 do upstream (idêntico). Os demais li localmente sem comparar com o upstream.

Testes dos `kin_*`: ver seção 8.

---

## 4. Unidades Python

### 4.1 `klippy/toolhead.py` (614)
- Classes: `Move`, `LookAheadQueue`, `ToolHead`, `ToolHeadCommandHelper`, `add_printer_objects`.
- Deps klippy: `mcu`, `chelper`, `kinematics.extruder`, `motion_queuing` (extra), `gcode` (Coord), `pins`, módulos carregados: `gcode_move`, `homing`, `idle_timeout`, `statistics`, `manual_probe`, `tuning_tower`, `garbage_collection`, mais `kinematics.<nome>` por `importlib` dinâmico. Terceiros: nenhum.
- Contratos:
  - G-code: `G4 P`, `M400`, `SET_VELOCITY_LIMIT VELOCITY ACCEL SQUARE_CORNER_VELOCITY MINIMUM_CRUISE_RATIO` (resposta `max_velocity: %.6f\nmax_accel: %.6f\nminimum_cruise_ratio: %.6f\nsquare_corner_velocity: %.6f`), `M204 S` ou `P`/`T` (usa o mínimo).
  - Config `[printer]`: `kinematics`, `max_velocity`, `max_accel`, `minimum_cruise_ratio` (0.5), `square_corner_velocity` (5.0); mais as opções que cada kinematics lê (`max_z_velocity`, `max_z_accel`).
  - `get_status` (objeto `toolhead`): `print_time`, `stalls`, `estimated_print_time`, `extruder`, `position` (Coord), `max_velocity`, `max_accel`, `minimum_cruise_ratio`, `square_corner_velocity`, `extra_axes` (dict nome para índice) + `homed_axes`, `axis_minimum`, `axis_maximum` (+ extras do kinematics, p.ex. `cone_start_z` no delta).
  - Eventos emitidos: `toolhead:sync_print_time(curtime, est_print_time, print_time)`, `toolhead:set_position`, `toolhead:manual_move`, `toolhead:update_extra_axes`. Escuta `klippy:shutdown`.
  - API para outros módulos: `get_position/set_position/move/manual_move/dwell/wait_moves/get_last_move_time/flush_step_generation/register_lookahead_callback/limit_next_junction_speed/drip_move/get_trapq/get_kinematics/get_extruder/set_extruder/add_extra_axis/remove_extra_axis/get_extra_axes/get_max_velocity/set_max_velocities/stats/check_busy/get_status`. Interface "extra axis" (`check_move`, `calc_junction`, `process_move`, `get_name`, `get_axis_gcode_id`).
  - Linha de log parseada: `stats()` retorna `"print_time=%.3f buffer_time=%.3f print_stall=%d"` (agregada por `statistics`; lida por `scripts/graphstats.py` `[INFERÊNCIA]`).
- Traços:
  - Acoplado ao reactor com greenlets: `reactor.pause` em `_check_pause`, `wait_moves` e (via motion_queuing) `_await_flush_time`; `with reactor.assert_no_pause()` protege o laço de `trapq_append`; timer `priming_timer`.
  - `kinematics` carregado por `importlib` com o nome vindo da config.
  - Matemática double sensível (seção 5): `math.sqrt`, `**2` (libm `pow`), `sum([...])` (CPython >= 3.12 usa soma compensada de Neumaier em floats; versões antigas não, então o resultado de `move_d` pode diferir entre versões do Python), `min/max`, divisões que em Python levantam `ZeroDivisionError` onde Rust devolveria inf/NaN.
  - Estado de máquina "NeedPrime" / "Priming" / "" e timer.
  - `Move` é lido por vários módulos (`axes_d`, `axes_r`, `move_d`, `max_cruise_v2`, `accel`, `is_kinematic_move`, `limit_speed`, `move_error`, `start_pos`, `end_pos`, `accel_t`, `cruise_t`, `decel_t`, `start_v`, `cruise_v`).
- Testes: todo `.test` que move algo (`printers.test` com `move.gcode`, `commands.test` cobre `SET_VELOCITY_LIMIT` e `M204`, `out_of_bounds.test` cobre `move_error`).
- Fork: `BUFFER_TIME_HIGH = 2.0` (upstream 1.0). Nas regiões que comparei com o upstream (linhas 1 a 30 e 150 a 520) é a única diferença; as linhas 30 a 150 (`Move`, início de `LookAheadQueue`) e 520 a 614 não foram comparadas. Commit provável: `e47b094` e/ou `4eec735` `[INFERÊNCIA]`.
- Dificuldade: **alta**. Numérica fina + acoplamento ao reactor + eventos.

### 4.2 `klippy/stepper.py` (475)
- `MCU_stepper`, `PrinterStepper`, `parse_gear_ratio`, `parse_step_distance`, `GenericPrinterRail`, `LookupRail`, `LookupMultiRail`.
- Deps: `chelper`, `motion_queuing` (`allocate_syncemitter`), `pins` (`lookup_pin`, `setup_pin('endstop')`), `query_endstops`, `stepper_enable`, `force_move`, `motion_report`, `configfile` (`deprecate_mcu_code`), mcu (`create_oid`, `register_config_callback`, `lookup_command`, `lookup_query_command`, `seconds_to_clock`, `get_constants`, `print_time_to_clock`, `estimated_print_time`).
- Contratos MCU: `config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%u`; `reset_step_clock oid=%d clock=0` (on_restart); comandos `queue_step oid=%c interval=%u count=%hu add=%hi`, `set_next_step_dir oid=%c dir=%c`, `reset_step_clock oid=%c clock=%u`; consulta `stepper_get_position oid=%c` com resposta `stepper_position oid=%c pos=%i`. Constantes lidas: `STEPPER_STEP_BOTH_EDGE`, `STEPPER_BOTH_EDGE`, `STEPPER_OPTIMIZED_UNSTEP`. `invert_step=-1` liga step em ambas as bordas. `MAX_STEPCOMPRESS_ERROR=0.000025` convertido com `seconds_to_clock` (`int(t*freq)`) para `max_error_ticks`. Config: `step_pin`, `dir_pin`, `rotation_distance`, `gear_ratio` (`a:b, c:d`), `microsteps`, `full_steps_per_rotation` (200, múltiplo de 4), `step_pulse_duration` (<=1 ms), e no rail `endstop_pin`, `position_endstop`, `position_min`, `position_max`, `homing_speed`, `second_homing_speed`, `homing_retract_speed`, `homing_retract_dist`, `homing_positive_dir` (inferido por 1/4 do curso; chama `config.getboolean` para registrar o valor inferido).
- Eventos: escuta `klippy:connect` (consulta posição); emite `stepper:set_dir_inverted`, `stepper:sync_mcu_position`.
- Matemática para inteiros: `get_mcu_position`: `int(x+0.5)` se `x>=0`, senão `int(x-0.5)`; `_set_mcu_position` guarda `offset = mcu_pos*step_dist - commanded_pos`; `get_past_mcu_position(print_time)`: `int(print_time*freq)` para clock, depois `stepcompress_find_past_position`.
- Traços: despacho dinâmico `getattr(ffi_lib, alloc_func)(*params)`; ponteiros crus de `sk` e `trapq` guardados no objeto; `set_stepper_kinematics` troca `sk` preservando a posição MCU.
- Testes: todos; `multi_z.test`, `z_tilt.test` usam `LookupMultiRail` e `set_trapq(None)`.
- Fork: nenhum marcador; não comparei com o upstream.
- Dificuldade: média.

### 4.3 `klippy/kinematics/` (16 arquivos; todos expõem `load_kinematics(toolhead, config)`)
Interface comum esperada pelo toolhead e extras: `get_steppers()`, `calc_position(stepper_positions)`, `set_position(newpos, homing_axes)`, `clear_homing_state(axes)`, `home(homing_state)`, `check_move(move)`, `get_status(eventtime)`; alguns têm `update_limits`, `home_axis`, `get_calibration`. `get_status` devolve `homed_axes`, `axis_minimum`, `axis_maximum` (Coord).

| Arquivo (linhas) | Config lida | Detalhes | Testes | Dificuldade |
|---|---|---|---|---|
| `__init__.py` (5) | | só cabeçalho | | trivial |
| `none.py` (30) | | todos os métodos vazios | nenhum teste conhecido | baixa |
| `cartesian.py` (129) | `[stepper_x/y/z]` via `LookupMultiRail`; `[dual_carriage]` (`axis` x|y, `safe_distance`); `max_z_velocity`, `max_z_accel` | `limits=[(1.0,-1.0)]*3` marca "não homed"; `home_axis` usa `forcepos` com fator 1.5; usa `idex_modes.DualCarriages` | `printers.test`, `commands.test`, `dual_carriage.test` e quase todos | baixa a média |
| `corexy.py` (95) | idem, sem dual carriage | `calc_position = [0.5*(a+b), 0.5*(a-b), z]`; steppers de X e Y compartilham endstops cruzados | `printers.test`, `corexyuv.test`, `corexyuv.cfg` | baixa |
| `corexz.py` (95) | idem | `calc_position = [0.5*(a+c), y, 0.5*(a-c)]` | `printers.test` | baixa |
| `hybrid_corexy.py` (129) | + `[dual_carriage]` | calcula `x` por `pos[0]+pos[1]` ou `pos[3]-pos[1]`; consulta `dc_module.get_status()['carriage_1']` | `hybrid_corexy_dual_carriage.test`, `printers.test` | média |
| `hybrid_corexz.py` (129) | idem | análogo com Z | `printers.test` | média |
| `delta.py` (241) | `[printer] delta_radius`, `print_radius`, `minimum_z_position`; por stepper `arm_length`, `angle` | `mathutil.trilateration`; classe `DeltaCalibration` (usada por `delta_calibrate` extra) com `save_state` que escreve `SAVE_CONFIG` (`configfile.set`) e `respond_info` multi-linha | `delta.test`, `delta_calibrate.test`, `printers.test` | média |
| `deltesian.py` (184) | `min_angle`, `print_width`, `slow_ratio`, `minimum_z_position`, `arm_x_length`, `arm_length` | trilateração própria 2D; `check_move` detecta movimento de homing | `printers.test` | média |
| `polar.py` (151) | `max_angular_velocity` + steppers `bed`, `arm`, `z` | `distance_to_center`; `units_in_radians=True` no `stepper_bed` | `polar.test` | média |
| `rotary_delta.py` (227) | `shoulder_radius`, `shoulder_height`, `upper_arm_length`, `lower_arm_length`, `angle` | `RotaryDeltaCalibration` aloca `sk` em C só para `itersolve_calc_position_from_coord`; `save_state` | `printers.test`, `rotary_delta_calibrate.test` | média |
| `winch.py` (57) | `stepper_a..z`, `anchor_x/y/z` | homing não implementado; `trilateration` | `printers.test` | baixa |
| `generic_cartesian.py` (396) | `[carriage X]`, `[dual_carriage X]`, `[extra_carriage X]`, `[stepper X]` com `carriages:` | G-code `SET_STEPPER_CARRIAGES STEPPER CARRIAGES DISABLE_CHECKS`; `mathutil.gaussian_solve/pseudo_inverse/mat_mat_mul` para validar independência de eixos e calcular `calc_position`; `copy.deepcopy` | `generic_cartesian*.test` | média a alta |
| `kinematic_stepper.py` (57+) | `carriages:` (gramática `+c`, `-c`, `2.0*c`) | parser com `re`; chama `generic_cartesian_stepper_set_coeffs` em runtime | `generic_cartesian*.test` | média |
| `idex_modes.py` (399) | `safe_distance` | G-code `SET_DUAL_CARRIAGE CARRIAGE MODE=INACTIVE|PRIMARY|COPY|MIRROR`, `SAVE_DUAL_CARRIAGE_STATE NAME`, `RESTORE_DUAL_CARRIAGE_STATE NAME MOVE MOVE_SPEED`; objeto `dual_carriage` (`add_object`), evento emitido `dual_carriage:update_kinematics` (input_shaper escuta), status `carriages`, `carriage_0/1` | `dual_carriage.test`, `corexyuv.test`, `generic_cartesian*.test`, `hybrid_corexy_dual_carriage.test` | alta |
| `extruder.py` (319) | ver 4.4 | | | média a alta |

Nota: o contador de linhas de `kinematic_stepper.py` é 92, não 57 (57 é `winch.py`); a tabela acima abrevia, o valor correto é `kinematic_stepper.py` = 92.

Fork nos kinematics: nenhum marcador; não comparados com o upstream.

### 4.4 `klippy/kinematics/extruder.py` (319)
- Classes: `ExtruderStepper` (stepper + `extruder_stepper_alloc` + PA), `PrinterExtruder` (heater, trapq próprio, `check_move`, `calc_junction`, `process_move`), `DummyExtruder`.
- Deps: `stepper`, `chelper`, `heaters` (`setup_heater`), `motion_queuing`, `toolhead`, `gcode`.
- G-code: `SET_PRESSURE_ADVANCE [EXTRUDER] ADVANCE SMOOTH_TIME`, `SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER DISTANCE`, `SYNC_EXTRUDER_MOTION EXTRUDER MOTION_QUEUE`, `M104`, `M109`, `ACTIVATE_EXTRUDER EXTRUDER`.
- Config: `pressure_advance` (0), `pressure_advance_smooth_time` (0.040, até 0.200), `nozzle_diameter`, `filament_diameter`, `max_extrude_cross_section` (4*nozzle²), `max_extrude_only_velocity/accel/distance` (50), `instantaneous_corner_velocity` (1.0), mais tudo do stepper e do heater.
- `get_status` (extruder): campos do heater + `can_extrude` + `pressure_advance`, `smooth_time`, `motion_queue`.
- Mensagens de erro parseadas por UIs: `"Extrude below minimum temp\nSee the 'min_extrude_temp' config option for details"`, `"Extrude only move too long (%.3fmm vs %.3fmm)..."`, `"Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)..."`; log `"Extruder max_extrude_ratio=%.6f"`.
- Traços: `trapq_append(... start_pos[ea_index], 0., 0., 1., can_pressure_advance, 0., start_v, cruise_v, accel)` (o `can_pressure_advance` é bool convertido pelo cffi para double 0.0/1.0); mudança de `smooth_time` exige `flush_step_generation()` e `check_step_generation_scan_windows()`; mudança só de PA usa `register_lookahead_callback` para entrar no `print_time` exato do movimento; `ffi.gc(extruder_stepper_free)`.
- Testes: `extruders.test`, `pressure_advance.test`, `commands.test`, `corexyuv.test`, `dual_carriage.test`, `generic_cartesian_itex/iqex.test`.
- Fork: nenhum marcador. A impressora usa `pressure_advance = 0.030`.
- Dificuldade: média a alta (a parte difícil é o C do PA).

### 4.5 `klippy/extras/motion_queuing.py` (509)
- Papel: dono dos `trapq`, do `steppersyncmgr`, dos `syncemitter` e do timer de flush em segundo plano; controla `last_flush_time`, `last_step_gen_time`, `need_flush_time`, `need_step_gen_time`, `kin_flush_delay`; `drip_update_time`.
- API usada por outros: `allocate_trapq`, `wipe_trapq`, `lookup_trapq_append`, `allocate_syncemitter(mcu, name, alloc_stepcompress)`, `setup_mcu_movequeue`, `register_flush_callback(cb, can_add_trapq)`, `unregister_flush_callback`, `get_kin_flush_delay`, `check_step_generation_scan_windows`, `flush_all_steps`, `calc_step_gen_restart`, `note_mcu_movequeue_activity(mq_time, is_step_gen=True)`, `drip_update_time`, `check_drip_timing`, `stats`.
- Constantes: `BGFLUSH_LOW_TIME 0.200`, `BGFLUSH_HIGH_TIME 0.400`, `BGFLUSH_SG_LOW_TIME 1.000` (upstream 0.450), `BGFLUSH_SG_HIGH_TIME 1.250` (upstream 0.700), `BGFLUSH_EXTRA_TIME 0.250`, `MOVE_HISTORY_EXPIRE 30.`, `MIN_KIN_TIME 0.100`, `STEPCOMPRESS_FLUSH_TIME 0.050`, `SDS_CHECK_TIME 0.001`, `DRIP_SEGMENT_TIME 0.050`, `DRIP_TIME 0.100`.
- Erro: `"Internal error in stepcompress"` com `invoke_shutdown(msg, details)` onde `details = {queue_name, flush_time, step_gen_time, last_flush_time, last_step_gen_time, clear_history_time}` (consumido por `motion_report._handle_analyze_shutdown`).
- Traços: timers do reactor, `reactor.pause`, `assert_no_pause`; modo arquivo usa `_flush_handler_debug` (determinístico: lotes de 0.25 s até `need_flush_time - 1.5`); `stats()` chama `mcu.calibrate_clock` para cada MCU e recalibra `steppersync_set_time(offset, freq)`; `clear_history_time = max(0, est_print_time - 30)`.
- Testes: indiretos em todos.
- Fork: **o ponto principal do fork em movimento**. (a) Constantes `SG_LOW/HIGH` maiores (seção 6). (b) Instrumentação `[DEBUG-motion] version=host-v4`: `DebugFlushBatch`, anéis (`DEBUG_RING_SIZE 32`, slow 16, wake 48), `_flush_handler` virou wrapper que chama `_flush_handler_body` (corpo idêntico ao `_flush_handler` do upstream), `_advance_flush_time` com cronometragem por fase (callbacks, gen, finalize) em `try/finally`, `_debug_shutdown` registrado em `klippy:analyze_shutdown`, uso de `reactor.get_timing_debug_report` (fork). Linhas de log `[DEBUG-motion] motion batch ...`, `motion wake ...`, `motion flush stage=...`. Comparado com o upstream master por leitura completa.
- Dificuldade: alta (política de temporização; reproduzível só em modo arquivo).

### 4.6 `klippy/extras/input_shaper.py` (210) + `shaper_defs.py` (197)
- `input_shaper.py`: `InputShaperParams`, `AxisInputShaper`, `InputShaper`. G-code `SET_INPUT_SHAPER` com `SHAPER_TYPE[_X|_Y|_Z]`, `SHAPER_FREQ_X/Y/Z`, `DAMPING_RATIO_X/Y/Z`; resposta por eixo `shaper_type_x:mzv shaper_freq_x:38.500 damping_ratio_x:0.100000`. Config `[input_shaper]`: `shaper_type`, `shaper_type_x/y`, `shaper_freq_x/y/z`, `damping_ratio_x/y/z` (0.1). Eventos: escuta `klippy:connect` e `dual_carriage:update_kinematics`. Ao conectar cria um `input_shaper` sk por stepper com `trapq`, troca `stepper.set_stepper_kinematics(is_sk)`, chama `input_shaper_set_sk(is_sk, orig_sk)`, `input_shaper_set_shaper_params(sk, axis, n, A, T)` (passa `list` de floats; cffi converte para `double[]`), `input_shaper_update_sk`, e `motion_queuing.check_step_generation_scan_windows()`. Sem `get_status`. Com `dual_carriage` presente proíbe a seção `[input_shaper]` ativa (erro de config) e a configuração vai por `SET_INPUT_SHAPER` depois de selecionar o carro.
- `shaper_defs.py`: formas `zv`, `mzv` (`mzv(n,t)` e `mzv(n, tau=...)` com parsing por regex `(\w+)\s*\((.*)\)$`), `zvd`, `ei`, `2hump_ei`, `3hump_ei`. Usa `math.sqrt/exp/pi/cos/sin` e `mathutil.pseudo_inverse` (eliminação gaussiana em Python puro) no `mzv`. `SHAPER_VIBRATION_REDUCTION=20`, `DEFAULT_DAMPING_RATIO=0.1`. Constantes mágicas dos polinômios de EI.
- Traços: A e T chegam ao MCU via C (seção 5); `exp`, `cos`, `sin` dependem da libm; regex com argumentos posicionais ou nomeados misturados, `int` vs `float` por presença de ponto.
- Dependências fora do recorte: `resonance_tester`, `shaper_calibrate` (numpy) importam `shaper_defs`.
- Testes: `input_shaper.test` (`mzv(5,0.6)`, `2hump_ei`), `generic_cartesian*.test`, `hybrid_corexy_dual_carriage.test`.
- Fork: sem marcador no código. O commit `78b0528 feat(InputShaper): Get the input shaper working` corresponde a `config/ender-3-v3-ke/input_shaper.cfg` `[INFERÊNCIA pelo assunto e pela existência do arquivo]`.
- Dificuldade: média.

### 4.7 `klippy/extras/gcode_move.py` (294)
- Comandos: `G0/G1 X Y Z E F`, `G20` (erro), `G21`, `M82`, `M83`, `G90`, `G91`, `G92`, `M114`, `M220 S`, `M221 S`, `SET_GCODE_OFFSET X Y Z E [_ADJUST] MOVE MOVE_SPEED`, `SAVE_GCODE_STATE NAME`, `RESTORE_GCODE_STATE NAME MOVE MOVE_SPEED`, `GET_POSITION` (resposta multi-linha `mcu:/stepper:/kinematic:/toolhead:/gcode:/gcode base:/gcode homing:`).
- `get_status`: `speed_factor`, `speed`, `extrude_factor`, `absolute_coordinates`, `absolute_extrude`, `homing_origin`, `position`, `gcode_position`, `axis_map`.
- Eventos escutados: `klippy:ready`, `klippy:shutdown`, `klippy:analyze_shutdown`, `toolhead:set_position`, `toolhead:manual_move`, `toolhead:update_extra_axes`, `gcode:command_error`, `extruder:activate_extruder`, `homing:home_rails_end`.
- Extensões: `set_move_transform(transform, force)` usado por `bed_mesh`, `bed_tilt`, `exclude_object`, `skew_correction`, `tuning_tower`, `z_thermal_adjust` (e `axis_twist_compensation` na KE via bed_mesh/outros `[INFERÊNCIA]`).
- Traços: `getattr(self, 'cmd_' + cmd)` para registrar; `axis_map` dinâmico para eixos extras; velocidade em mm/s = F/60.
- Testes: `commands.test`, `printers.test` (`move.gcode`), `macros.test` (G92 em macro), `gcode_arcs.test`.
- Fork: nenhuma diferença funcional encontrada; `_handle_analyze_shutdown` existe no upstream (é parte do mecanismo `klippy:analyze_shutdown`).
- Dificuldade: média.

### 4.8 `klippy/extras/homing.py` (358)
- `HomingMove`, `Homing`, `PrinterHoming`. G-code `G28 [X] [Y] [Z]`. Eventos emitidos: `homing:homing_move_begin`, `homing:homing_move_end`, `homing:home_rails_begin`, `homing:home_rails_end`. API: `probing_move(mcu_probe, pos, speed, check_movement)`, `manual_home(...)`.
- Constantes: `HOMING_START_DELAY 0.001`, `ENDSTOP_SAMPLE_TIME .000015`, `ENDSTOP_SAMPLE_COUNT 4`. `rest_time = move_t / max_steps` vai para `mcu_endstop.home_start`.
- Fluxo: `flush_step_generation`, `home_start` em cada endstop (trsync), `dwell(HOMING_START_DELAY)`, `drip_move` (bloqueia até a `completion`), `home_wait`, `flush_step_generation`, `get_past_mcu_position(trigger_time)` (usa `stepcompress_find_past_position`), calcula `haltpos`/`trigpos` por `kin.calc_position`, `toolhead.set_position`. Erros: `"No trigger on %s after full movement"`, `"Endstop %s still triggered after retract"`, `"Probe triggered prior to movement"`, `"Homing failed due to printer shutdown"`.
- Traços: usa `reactor.register_callback`/`Completion` de forma assíncrona (`multi_complete`), dispara múltiplos MCUs; interação direta com `mcu.py` (`MCU_endstop`, `MCU_trsync`, `trdispatch`).
- Testes: quase todos começam com `G28`; `z_virtual_endstop.test`, `bltouch.test`, `smart_effector.test`, `eddy.test`, `load_cell.test` cobrem variantes de probe. Em modo arquivo `home_wait` retorna imediatamente (o caminho com trigger real não é exercitado).
- Fork: sem marcador; cabeçalho de copyright indica 2016-2026 (arquivo recente do upstream).
- Dificuldade: alta (assíncrono, multi-MCU, depende de `mcu.py` e `probe`).

### 4.9 `klippy/extras/force_move.py` (140)
- G-code `STEPPER_BUZZ STEPPER` (sempre registrado por stepper), e se `enable_force_move: True`: `FORCE_MOVE STEPPER DISTANCE VELOCITY ACCEL` e `SET_KINEMATIC_POSITION X Y Z SET_HOMED CLEAR_HOMED` (alias `CLEAR`). Logs: `"Stepper buzz %s"`, `"FORCE_MOVE %s distance=%.3f velocity=%.3f accel=%.3f"`, `"SET_KINEMATIC_POSITION pos=%.3f,%.3f,%.3f set_homed=%s clear_homed=%s"`.
- Mecanismo: aloca seu próprio `trapq` e um `cartesian_stepper_alloc(b'x')`, troca `sk`/`trapq` do stepper (`set_stepper_kinematics`, `set_trapq`), `trapq_append`, `note_mcu_movequeue_activity`, `dwell`, `flush_step_generation`, restaura e `wipe_trapq`. `calc_move_time(dist, speed, accel)` é reutilizada por `manual_stepper`.
- A KE deixa `[force_move]` comentado em `printer.cfg`; mas `stepper.PrinterStepper` carrega `force_move` para registrar `STEPPER_BUZZ`.
- Testes: `STEPPER_BUZZ` em `delta.test`, `dual_carriage.test`, `corexyuv.test`, `manual_stepper.test`, `generic_cartesian.test`, `hybrid_corexy_dual_carriage.test`.
- Dificuldade: baixa.

### 4.10 `klippy/extras/manual_stepper.py` (234)
- G-code `MANUAL_STEPPER STEPPER ENABLE SET_POSITION SPEED ACCEL MOVE SYNC STOP_ON_ENDSTOP (home|probe|try_*|inverted_*|legado -2/-1/1/2) GCODE_AXIS INSTANTANEOUS_CORNER_VELOCITY LIMIT_VELOCITY LIMIT_ACCEL`. Config `[manual_stepper X]`: `velocity` (5), `accel` (0), `position_min/max`, `endstop_pin`.
- Registra-se como "eixo extra" do toolhead (`add_extra_axis`/`remove_extra_axis`, `process_move`, `check_move`, `calc_junction`, `get_axis_gcode_id`). Também implementa a interface de "toolhead" para homing (`flush_step_generation`, `get_position`, `set_position`, `get_last_move_time`, `dwell`, `drip_move`, `get_kinematics`, `get_steppers`, `calc_position`).
- Trapq próprio, `cartesian_stepper_alloc(b'x')`.
- Testes: `manual_stepper.test` (+ `.cfg`).
- Dificuldade: média.

### 4.11 `klippy/extras/motion_report.py` (267)
- Endpoints API (via `bulk_sensor.BatchBulkHelper`): `motion_report/dump_stepper` (header `interval, count, add`; payload `data, start_position, start_mcu_position, step_distance, first_clock, first_step_time, last_clock, last_step_time`), `motion_report/dump_trapq` (header `time, duration, start_velocity, acceleration, start_position, direction`).
- `get_status`: `live_position`, `live_velocity`, `live_extruder_velocity`, `steppers`, `trapq` (atualiza no máximo a cada 0.250 s).
- Escuta `klippy:connect` e `klippy:analyze_shutdown`; ao shutdown de MCU ou "Internal error in stepcompress" imprime os `queue_step` e trapqs próximos (`"Dumping stepper '%s' (%s) %d queue_step:"`, `"queue_step %d: t=%d p=%d i=%d c=%d a=%d"`, `"Dumping trapq '%s' %d moves:"`, `"move %d: pt=%.6f mt=%.6f sv=%.6f a=%.6f sp=(...) ar=(...)"`, `"Requested toolhead position at shutdown time %.6f: %s"`, `"stepcompress error info: %s"`).
- Usa `trapq_extract_old` e `stepcompress_extract_old` (structs compartilhadas).
- Testes: carregado por todo stepper; endpoints não exercitados nos testes.
- Fork: nenhuma diferença encontrada.
- Dificuldade: média (depende de `bulk_sensor` e do webhooks).

### 4.12 `klippy/extras/extruder_stepper.py` (24), `stepper_enable.py` (147)
- `extruder_stepper`: `[extruder_stepper X]` com `extruder:`; `sync_to_extruder` no `klippy:connect`. Dificuldade baixa. Testes: `extruders.test`.
- `stepper_enable`: G-code `M18`, `M84`, `SET_STEPPER_ENABLE STEPPER ENABLE`; `get_status` `{steppers: {nome: bool}}`; usa `stepper.add_active_callback` (que registra um flush callback em `motion_queuing` e liga o motor no primeiro `print_time` ativo), `DISABLE_STALL_TIME 0.100`; evento `stepper_enable:motor_off`, escuta `gcode:request_restart`. Compartilha pino de enable entre steppers por contagem. Testes: `commands.test` (`M18`), `manual_stepper.test` (`M84`). Dificuldade baixa a média.

### 4.13 `klippy/mathutil.py` (220), só o que o movimento usa
- `trilateration` (delta, winch, rotary_delta, deltesian parcial), `matrix_*`, `mat_transp`, `mat_mat_mul`, `mat_mul_transp`, `gaussian_solve`, `pseudo_inverse`, `solve_linear_equations`. Python puro, sem numpy. Fora do recorte: `coordinate_descent`, `background_coordinate_descent` (usa `multiprocessing` e `queuelogger`). Dificuldade baixa, mas a ordem de soma e a eliminação gaussiana determinam bits (alimenta `A`/`T` do `mzv`).

---

## 5. Pontos flutuantes cujo resultado vira inteiro enviado ao MCU

Regras gerais para o Rust (valem para todas as linhas abaixo):
- `f64` IEEE-754 round-to-nearest-even, nunca `f32`, nunca `mul_add`/`fma`, nunca reordenar expressões (o Rust não faz contração; o gcc pode fazer em alvos com FMA, ver 5.1).
- `sqrt` é exato em IEEE (`f64::sqrt` serve). `atan2`, `sin`, `cos`, `exp`, `pow` NÃO são exatos e variam por libm.
- Casts f64 para inteiro em C truncam em direção a zero; o Rust `as` também trunca (e satura, ao contrário do C que é UB fora de faixa).

### 5.1 Ambiente de compilação do C de referência
- x86-64: `-mfpmath=sse -msse2` (fornecido por `check_gcc_option`), sem `-mfma`, portanto sem FMA; resultados reproduzíveis.
- A KE (MIPS) compila o `.so` com o gcc do dispositivo sem as flags SSE. Se o alvo/gcc contrair `a*b+c` em madd/FMA o resultado difere do x86. `[INFERÊNCIA]`: não verifiquei o gcc nem o ISA da placa; cheque com `objdump -d c_helper.so | grep -E 'madd|msub|nmadd|nmsub|fma'` no `.so` que roda na KE e decida qual build é o oráculo.
- Python: `reactor.monotonic` usa `CLOCK_MONOTONIC_RAW`, não `Instant` do Rust.

### 5.2 Tabela

| # | Cálculo | Onde | O que preservar |
|---|---|---|---|
| F1 | Planejamento: `move_d = sqrt(sum([d*d ...]))`, `inv_move_d`, `axes_r`, `min_move_t`, `max_cruise_v2 = velocity**2`, `delta_v2 = 2.0*move_d*accel`, `mcr_delta_v2`, junction (`sqrt`, `R_jd`, `quarter_tan_theta_d2`), `set_junction` (`accel_d = (cruise_v2-start_v2)*(.5/accel)`, `accel_t = accel_d/((start_v+cruise_v)*0.5)`) | `toolhead.py` Move/LookAheadQueue | Ordem esquerda para direita como escrita. `x**2` em Python chama `pow(x,2.0)` da libm, que não é garantidamente igual a `x*x` (glibc >= 2.28 tem erro < 1 ulp); validar por teste de propriedade contra CPython antes de trocar por `x*x`. `sum()` de CPython >= 3.12 usa soma compensada; fixar a versão do Python do oráculo ou reproduzir ambas. `min`/`max` com semântica Python (NaN). Constantes `999999999.9`, `99999999.9`. `ZeroDivisionError`/`ValueError` do Python não existem em IEEE. |
| F2 | `next_move_time = (next_move_time + accel_t + cruise_t + decel_t)`; `trapq_append` | `toolhead._process_lookahead`, `trapq.c` | Soma associativa à esquerda. `half_accel = .5*accel` e `-.5*accel` no decel; `start_pos` do segmento seguinte vem de `move_get_coord` do anterior (não de `start_pos + ...` acumulado de outro jeito). `move_get_distance = (start_v + half_accel*t)*t`; `coord = start + axes_r*dist`. |
| F3 | Posição do stepper `calc_position_cb(sk, m, t)` | `kin_*.c` | Ordem por fórmula na seção 3.12. Delta: `(arm2 - dx*dx) - dy*dy`. Generic: `(ax*cx + ay*cy) + az*cz`. |
| F4 | Solver: `next_time = (old.time*guess_dist - guess.time*og_dist)/(guess_dist - og_dist)`, bisseção `(low+high)*.5`, `high_time = 2.*high_time - last_time`, `seek_time_delta = 1.5*(guess.time - last_time)`, limiares `1e-9`, `half_step+half_step+1e-8`, `target ± (half_step+half_step)` | `itersolve.c` | Candidato nº 1 a FMA no gcc (`a*b - c*d`). Comparações `>`/`<` com NaN: `!(next_time > low && next_time < high)` trata NaN como fora; manter. `sk->commanded_pos = target - (sdir ? half_step : -half_step)` ao fim de cada trecho. |
| F5 | Tempo para relógio: `offset = print_time - last_step_print_time; rel_sc = (step_time + offset)*mcu_freq; step_clock = last_step_clock + (uint64_t)rel_sc` | `stepcompress_append` | Associatividade `(step_time + (print_time - lspt))*freq`. `lspt = mcu_time_offset + (lsc - .5)/mcu_freq` com `lsc` convertido de u64 para double. Cast f64 para u64 com valor negativo é UB em C: definir comportamento (recomendo wrap como x86-64: `rel_sc as i64 as u64`) e cobrir com teste. |
| F6 | Filtro step+dir+step: `diff = (double)(int64)(step_clock - next_step_clock)`, `diff < SDS_FILTER_TIME*mcu_freq` (`.000750`) | idem | Comparação em double; `.000750*freq` calculado em double. |
| F7 | Compressão: tudo `int32/uint32` (`compress_bisect_add`, `minmax_point`, `check_line`) | `stepcompress.c` | Aritmética modular de 32 bits e overflow signed de `nextmininterval*nextcount` e `add*nextaddfactor`: usar `wrapping_*` e provar ou testar ausência de overflow; `idiv_up/idiv_down` (negativos truncam para zero, floor emulado por `(n-d+1)/d`); `QUADRATIC_DEV 11`; `count > 0x200` interrompe; preferência `zerocount + zerocount/16 >= bestcount`; `add` int16 e `count` uint16 truncam ao serializar. Saída: `(interval, count, add)`. |
| F8 | `clock_from_time`: `(int64)((t - conv_time)*freq + .5) + conv_clock` | `msgblock.c`, usado por `steppersyncmgr_gen_steps` (flush_clock, clear_clock) e por `serialqueue` | Truncamento para zero após `+.5` (para negativo arredonda diferente de round-half-up); `conv_clock` u64 com wrap. |
| F9 | `queue_step` serializado: `add*addfactor + interval*(count-1)` em `uint32`; `last_clock = first_clock + ticks` | `add_move` | Wrap de 32 bits; VLQ de `encode_int` trata `uint32` como `int32` (negativos). |
| F10 | `find_past_position`: `ticks = (int32)(clock - first_clock) + interval`; `a = .5*add`, `b = interval - .5*add`, `c = -ticks`; `offset = (sqrt(b*b - 4*a*c) - b)/(2.*a)` atribuído a `int32` | `stepcompress.c` | Define a posição de disparo do endstop/probe, logo o Z medido. Truncamento do double para int. Quando `add == 0`, divisão inteira `ticks/interval`. |
| F11 | Python: `int(print_time*mcu_freq)`, `seconds_to_clock = int(t*freq)`, `max_error_ticks = int(0.000025*freq)`, `step_pulse_ticks`, `estimated_print_time = int(clock + (eventtime-sample)*freq)/mcu_freq` | `clocksync.py`, `mcu.py`, `stepper.py` | Truncamento de `int()`. Para MCU secundário `print_time_to_clock = int((pt - adj_offset)*adj_freq)`. `mcu_freq` do `steppersync_set_time` é `float(mcu.seconds_to_clock(1.))` na configuração e depois o par `(offset, freq)` de `calibrate_clock` (regressão no `clocksync`, fora do recorte). |
| F12 | `get_mcu_position`: `mcu_pos = (cmd_pos + offset)/step_dist; int(mcu_pos+0.5)` ou `int(mcu_pos-0.5)`; `_set_mcu_position` | `stepper.py` | Mesma regra (meio afastando de zero). `step_dist = rotation_dist/steps_per_rotation`, `steps_per_rotation = full_steps*microsteps*gearing`, `gearing = prod(g1/g2)`. |
| F13 | Shaper: `A, T` calculados em Python (`exp`, `sqrt`, `cos`, `sin`, `pseudo_inverse`), normalização `a*(1./sum_a)`, `shift_pulses`, convolução com `t += pulse.t ... t -= pulse.t` | `shaper_defs.py`, `kin_shaper.c` | `exp/cos/sin` dependem da libm (glibc vs musl vs crate `libm`); paridade bit a bit só se usar a mesma libm via FFI. Em C manter a soma e subtração sequenciais de `t`; `res += a*pos` na ordem: `i = p_ind-1 ... 0`, depois `p_ind ... n-1`. |
| F14 | Pressure advance: integrais com `half_v = .5*start_v`, `sixth_a = (1./3.)*half_accel`, `half_b = .5*base`, `third_v = (1./3.)*start_v`, `eighth_a = .25*half_accel`, `base += pa*start_v`, `start_v += pa*2.*half_accel`, `res += ...; res -= ...`, `area*inv_half_smooth_time2` | `kin_extruder.c` | Ordem exata das operações; `(1./3.)` avaliado como constante double; `list_prev_entry/next_entry` atravessando movimentos até cobrir `[t-hst, t+hst]`. |
| F15 | Polar `atan2`, rotary_delta `atan2/sin/cos` | `kin_polar.c`, `kin_rotary_delta.c` | libm. Aceitar tolerância de 1 ulp ou chamar a libm do sistema. Não afeta a KE. |
| F16 | Agendamento do flush (em segundos): `flush_time = max(want, last, want_sg - 0.05)`, etc. | `motion_queuing.py` | Só em modo ao vivo depende do relógio de parede; em modo arquivo `estimated_print_time` é a função constante 0 (`mcu.py` linha 1304), portanto determinístico. |

Efeito de agendamento sobre bytes: `queue_flush(sc, move_clock)` só comprime passos até `flush_clock`; portanto a segmentação dos `queue_step` (não os tempos de passo) depende dos instantes de flush. Em modo ao vivo isso varia de execução para execução; em modo arquivo (`-o`) é determinística.

---

## 6. Lookahead, planejamento, flush e commits `motion` do fork

### 6.1 Planejamento (toolhead.py)
- `Move`: `accel = max_accel`, `junction_deviation = scv²·(√2−1)/max_accel`, `mcr_pseudo_accel = max_accel·(1−minimum_cruise_ratio)`. Movimento só de extrusão (`move_d < 1e-9`): `is_kinematic_move=False`, `accel=99999999.9`, `velocity=speed` (sem teto `max_velocity`), `move_d = max(|axes_d[3:]|)`.
- `calc_junction(prev)`: `max_start_v2 = min(max_cruise_v2, prev.max_cruise_v2, prev.next_junction_v2, prev.max_start_v2 + prev.delta_v2, *extra_axis.calc_junction)`, depois limite de velocidade centrípeta aproximada com `sin_theta_d2`, `cos_theta_d2`, `R_jd`, `quarter_tan_theta_d2` (cada limite com `move_jd_v2`, `pmove_jd_v2`, `move_centripetal_v2`, `pmove_centripetal_v2`); `max_mcr_start_v2 = min(max_start_v2, prev.max_mcr_start_v2 + prev.mcr_delta_v2)`.
- `LookAheadQueue.add_move`: acumula `junction_flush -= move.min_move_t`; devolve `junction_flush <= 0` como pedido de flush. `flush()` zera `junction_flush = LOOKAHEAD_FLUSH_TIME (0.150)`.
- `LookAheadQueue.flush(lazy)`: passe reverso do último ao primeiro computando `start_v2 = min(max_start_v2, next_start_v2 + delta_v2)` (assume parada total após o último), e o ramo "minimum_cruise_ratio" (`mcr_start_v2`, `peak_cruise_v2`, `pending_cv2_assign`, `update_flush_count`) que decide quantos movimentos podem ser liberados em modo `lazy` (`flush_count = i + pending_cv2_assign`); passe direto propaga `cruise_v2` e chama `move.set_junction(min(start_v2,cruise_v2), cruise_v2, min(next_start_v2,cruise_v2))`. Em lazy sem pico, retorna lista vazia.
- Estados do toolhead: `special_queuing_state` em {"NeedPrime", "Priming", ""}. `_check_pause` roda quando `print_time > need_check_pause`: se NeedPrime/Priming atualiza o timer de priming (`PRIMING_CMD_TIME 0.100`), e em qualquer estado dorme (`reactor.pause`) enquanto `print_time − est_print_time > BUFFER_TIME_HIGH`; no estado principal adia a checagem (`need_check_pause = print_time`) e cede (`pause(NOW)`) para não matar de fome as outras tarefas. `_calc_print_time`: `print_time = max(est_print_time + BUFFER_TIME_START(0.250), calc_step_gen_restart)`. `print_stall` conta quando o gcode atrasa e o MCU ficou sem movimentos.
- `_handle_step_flush` (primeiro flush callback): se estado "" e `step_gen_time >= print_time − kin_flush_delay − 0.001`, chama `_flush_lookahead(is_runout=True)`.

### 6.2 Flush (motion_queuing.py)
- `note_mcu_movequeue_activity(t)` avança `need_step_gen_time` (+`kin_flush_delay`) e `need_flush_time`, e "chuta" o timer se estava parado.
- `_flush_handler_body`: se `last_step_gen_time < need_step_gen_time − 2·kin_flush_delay` (passos em curso): alvo `want_sg_time = est_print_time + SG_HIGH`; se `last_step_gen_time + (SG_HIGH−SG_LOW) > est_print_time` agrupa a partir do último ("run-to-run reproducibility", com tolerância 5 ms), limitado a `aggr_sg_time`; `_advance_flush_time(0., want_sg_time)`; reagenda em `last_step_gen_time − SG_LOW`. Senão (relaxado): `want_flush_time = min(est+0.400, need_flush_time+0.250)`, reagenda em `last_flush_time − 0.200`, e `NEVER` quando `last_flush_time >= need_flush_time + 0.250`.
- `_advance_flush_time`: `flush_time = max(want_flush, last_flush, want_sg − 0.050)`; `step_gen_time = max(want_sg, last_sg, flush_time)`; callbacks, `steppersyncmgr_gen_steps`, `trapq_finalize_moves(step_gen_time − kin_flush_delay, clear_history_time)`.
- `kin_flush_delay`: máximo de `gen_steps_pre_active/post_active` de todos os steppers com trapq (shaper: maior pulso; extruder com PA: `smooth_time/2`), mínimo `SDS_CHECK_TIME 0.001`; recomputado por `check_step_generation_scan_windows()` (chamado após input shaper, PA e sync de extruder).
- `drip_update_time`: desliga o timer, avança `start_time − 0.001`, loop de segmentos de `DRIP_SEGMENT_TIME 0.050` até `drip_completion.test()`, esperando `DRIP_TIME 0.100` à frente do relógio estimado; restaura timer.
- Modo arquivo: `_flush_handler_debug` ignora relógio de parede.

### 6.3 Diferenças do fork (verificadas contra o upstream master)
| Arquivo | Upstream | Fork | Efeito |
|---|---|---|---|
| `toolhead.py` `BUFFER_TIME_HIGH` | 1.0 | 2.0 | Janela inicial do lookahead depois de ocioso (`set_flush_time(2.0)`) dobra; `_check_pause` só segura o produtor G-code quando o buffer passa de 2 s; `will_pause_time` do priming também. O host pode empilhar até 2 s de trapq à frente do MCU. |
| `motion_queuing.py` `BGFLUSH_SG_LOW_TIME` | 0.450 | 1.000 | Passos em curso: o timer acorda quando falta 1.0 s de passos gerados. |
| `motion_queuing.py` `BGFLUSH_SG_HIGH_TIME` | 0.700 | 1.250 | Gera passos até 1.25 s à frente do relógio estimado. `batch_time = HIGH − LOW = 0.25` ficou igual. |
| `motion_queuing.py` instrumentação | ausente | `[DEBUG-motion] version=host-v4` | custo por lote de flush (chamadas `monotonic()`), anéis de histórico. |
| `itersolve.c`, `stepcompress.c`, `serialqueue.c` | ausente | `[DEBUG-motion] motion-v3/v4` | ver 3.3, 3.4, 3.6. |
| `reactor.py`, `virtual_sdcard.py`, `mcu.py`, `klippy.py` | ausente | `[DEBUG-motion]` | fora do meu recorte; `virtual_sdcard.work_handler` cede com `reactor.pause(NOW)` após cada leitura de 8192 bytes (linha ~423) com cronometragem. Não comparei se o `pause(NOW)` é do fork. |

### 6.4 Commits do fork (histórico local, `git log` não executado)
Ramo `ender-3-v3-ke`, HEAD `e47b094`. Pelo `.git/logs/HEAD` (hashes completos abaixo são os que o reflog lista; assuntos são Conventional Commits):

| Hash | Assunto | Relação com o recorte |
|---|---|---|
| `78b05289908b86391f92c0f32dba5ad4359b247f` | `feat(InputShaper): Get the input shaper working` | provável `config/ender-3-v3-ke/input_shaper.cfg` `[INFERÊNCIA]` |
| `7777d10424b40e51dde26aa15cf43655a4961d6b` | `feat(debug): preserve shutdown provenance` | `mcu.py`, `klippy.py` (instrumentação) |
| `aac1802d46ba93f142e8d6eae48db20b7ca711df` | `feat(debug): trace host motion scheduling` | `motion_queuing.py`, `reactor.py`, `virtual_sdcard.py` |
| `4eec735c3be688fcf561c2ce2a5765b3558eb510` | `fix(motion): tolerate host scheduling stalls` | candidato a mudar constantes de buffer (`toolhead.py` e/ou `motion_queuing.py`) |
| `3bc87895928711f799c4b0a535d3644ea86269cf` | `feat(debug): capture step compression state` | `stepcompress.c`, `itersolve.c` |
| `cf3c4ddb3cac7ae416f86349e1805ea8f193fa97` | `feat(debug): trace serial dispatch decisions` | `serialqueue.c` |
| `7d0860dbbded5d6b0285478478081d82168fa129` | `chore(ender3-ke): retain mainline diagnostic logs` | `config/ender-3-v3-ke` (scripts, `SERVICES.md`) |
| `e47b09436a7ef87ab02d2212714407ccca2c0f66` (HEAD) | `fix(motion): keep producer ahead of step generation` | candidato às constantes `SG_LOW/HIGH` e/ou `BUFFER_TIME_HIGH` |

O reflog também lista uma primeira versão abandonada dos mesmos assuntos (`36935fd`, `aef8165`, `67136ff`, `f43630a`, `5de84f3`, `e666233`, depois `reset --hard` para `152aa00`). A associação commit e arquivo é por assunto. Para confirmar, o agente principal deve rodar:
`git log --oneline -- klippy/toolhead.py klippy/chelper klippy/extras/motion_queuing.py klippy/stepper.py klippy/kinematics klippy/extras/input_shaper.py`
e `git show --stat 4eec735 e47b094`.
Não existe outro trabalho em Rust no repositório (`**/*.rs` e `**/Cargo.toml` vazios); `experiment/klippy-rs` aponta para `7c8d59f`, sem commits próprios.

---

## 7. O helper C pode ficar como está atrás de FFI?

Sim em termos de ABI, com custos. A superfície exportada (`defs_all`) é C simples: ponteiros opacos, `double`, inteiros, três structs de layout fixo, um callback de log. Um host em Rust pode ligar `c_helper` estaticamente (crate `cc`, que substitui `check_build_c_library`) ou por `dlopen` (`libloading`) e chamar as mesmas funções.

Custos e limites:
1. **Toolchain C permanece** na build e na cross-compilação para MIPS. A placa hoje compila no dispositivo; no Rust seria no CI. As flags `-flto -fwhole-program` exigem `__visible` em cada função exportada; qualquer função interna que o Rust precise chamar (`stepcompress_append`, `itersolve_generate_steps`, `serialqueue_send_batch`, `trapq_check_sentinels`) teria que ganhar `__visible` ou sair do whole-program.
2. **Linux apenas.** Fontes incluem `linux/can.h`, `sys/prctl.h`, `termios.h`. Não compila no host Windows; testes de FFI rodam em Linux/WSL/CI.
3. **Cortes viáveis (limites de módulo reais):**
   - Bloco A {`trapq`, `itersolve`, todos `kin_*`, `kin_shaper`, `kin_extruder`, `kin_idex`}: compartilham `struct move` em lista intrusiva (shaper e PA percorrem vizinhos com `list_prev_entry/list_next_entry`, itersolve percorre o trapq). Portar um subconjunto exige reimplementar o layout C em `#[repr(C)]` ou portar o bloco inteiro.
   - Um solver individual (`kin_*`) pode ser portado em Rust expondo `extern "C" fn calc_position(sk, m, t) -> f64` e um `struct stepper_kinematics` `#[repr(C)]` com `calc_position_cb` apontando para ele. Custo: uma chamada indireta por avaliação (poucas por passo).
   - Bloco B {`msgblock`, `stepcompress`, `steppersync`, `serialqueue`, `trdispatch`, `pollreactor`}: `struct queue_message` e `struct list_head` são compartilhados por todos. `itersolve` chama `stepcompress_append/commit/get_step_dir` diretamente; um stepcompress em Rust precisa exportar essas funções com a mesma assinatura (`#[no_mangle] extern "C"`) para o itersolve C continuar a funcionar.
4. **Pontes de ciclo de vida:** `ffi.gc` do Python hoje libera tudo; em Rust é preciso `Drop` explícito com a ordem correta (steppersync antes de serialqueue; `orig_sk` vivo enquanto `input_shaper`/`dual_carriage` o referenciarem).
5. **`errorf`** chama o callback registrado de qualquer thread C; em Rust o callback deve ser `extern "C"` thread-safe, sem GIL.
6. **Threads:** o C cria 1 thread pthread por syncemitter e 1 por serialqueue (nomes via `prctl`). Funciona igual chamado do Rust; o chamador deve ficar bloqueado durante `steppersyncmgr_gen_steps` como o Python fica (garantia de não-concorrência).
7. **Paridade:** manter o C elimina o risco numérico do bloco A e do stepcompress; o risco passa a ser as entradas calculadas em Rust (planejamento do toolhead, `A`/`T` do shaper) e a escolha do compilador/flags do C.
8. **Instrumentação do fork** permanece (custo de CPU na MIPS: ~100 bytes por passo em `stepcompress_append`, anéis em `serialqueue`); ao migrar decida se `[DEBUG-motion]` continua sendo contrato (o `SERVICES.md` do fork o documenta como diagnóstico; nenhum outro arquivo do repositório faz parsing dessas linhas).

---

## 8. Cobertura de testes

`scripts/test_klippy.py` roda `klippy.py <cfg> -i <gcode> -o _test_output -v -d <dict>` e só verifica código de saída (`SHOULD_FAIL` inverte). Não há valores esperados. Para paridade são necessários oráculos novos.

Oráculos viáveis (propostos, não existentes):
1. Modo arquivo (`-o`): `estimated_print_time` é 0 e o flush é determinístico (`_flush_handler_debug`), então o arquivo de saída do serialqueue (`SQT_DEBUGFILE`) é reproduzível byte a byte para um mesmo `.cfg` + `.dict` + gcode. Serve de golden para todo o pipeline (precisa congelar versão do Python por causa de `sum()`/`pow`).
2. Golden de módulo: harness Python que chama `chelper` com entradas fixas e despeja `(interval,count,add)` por stepper via `stepcompress_extract_old`, e `pull_move` via `trapq_extract_old`.
3. Registro dos argumentos de `trapq_append` (hex float) por movimento para validar só o planejamento do toolhead.

Mapa unidade para testes:

| Unidade | Testes |
|---|---|
| toolhead, stepper, gcode_move, homing, motion_queuing, motion_report, stepper_enable | todos os 37 `.test` (todos carregam `[printer]`); `commands.test` cobre `M18`, `G92`, `SET_GCODE_OFFSET`, `SET_VELOCITY_LIMIT`, `M204`, `SAVE/RESTORE_GCODE_STATE`, `RESTART`; `out_of_bounds.test` (`SHOULD_FAIL`) cobre `check_move` |
| kinematics cartesian | `printers.test` (centenas de cfgs), `commands.test`, vários |
| corexy / corexz | `printers.test` (`example-corexy`, `example-corexz`), `corexyuv.test` |
| hybrid_corexy / hybrid_corexz | `printers.test`, `hybrid_corexy_dual_carriage.test` |
| delta | `delta.test`, `delta_calibrate.test`, `printers.test` (+ vários kossel) |
| deltesian | `printers.test` (`example-deltesian`) |
| polar | `polar.test` |
| rotary_delta | `printers.test`, `rotary_delta_calibrate.test` |
| winch | `printers.test` (`example-winch`) |
| generic_cartesian, kinematic_stepper | `generic_cartesian.test`, `generic_cartesian_itex.test`, `generic_cartesian_iqex.test` |
| idex_modes / dual_carriage | `dual_carriage.test`, `corexyuv.test`, `generic_cartesian*.test`, `hybrid_corexy_dual_carriage.test` |
| extruder, PA | `extruders.test`, `pressure_advance.test`, `commands.test`, e os testes com múltiplos extruders |
| input_shaper, shaper_defs | `input_shaper.test`, `generic_cartesian*.test`, `hybrid_corexy_dual_carriage.test` |
| manual_stepper | `manual_stepper.test` |
| force_move | `STEPPER_BUZZ` em `delta.test`, `dual_carriage.test`, `corexyuv.test`, `manual_stepper.test`, `generic_cartesian.test`, `hybrid_corexy_dual_carriage.test`; `FORCE_MOVE`/`SET_KINEMATIC_POSITION` não aparecem nos testes |
| homing (probe) | `z_virtual_endstop`, `bltouch`, `smart_effector`, `eddy`, `load_cell`, `z_tilt`, `bed_mesh` |
| serialqueue, trdispatch (caminhos reais) | não testados (modo arquivo curto-circuita) |
| kinematics `none` | nenhum |

A KE usa só o que está coberto por `commands.test`, `extruders.test`, `pressure_advance.test`, `input_shaper.test`, `bed_mesh.test`, `bltouch.test`, `z_virtual_endstop.test`, `exclude_object.test` (cartesian + extruder + shaper + probe).

---

## 9. Riscos transversais e ordem de port

### Riscos
1. **Oráculo de float depende de ambiente:** compilador/flags do C (FMA), versão do CPython (`sum` compensado, `pow`), libm (`exp/sin/cos/atan2`). Escolher e congelar um oráculo (sugestão: x86-64, gcc com `-mfpmath=sse -msse2`, CPython fixo) e registrar o desvio permitido para a MIPS.
2. **UB e wrap no C:** overflow de `int32` em `compress_bisect_add`, `(uint64_t)` de double negativo, `list_prev_entry` além do sentinela de cabeça, `inline` não-estático em `trapq.c`. O Rust precisa decidir semântica explícita e testar nos limites.
3. **Concorrência:** 1 thread por stepper + thread serial + thread Python do `serialhdl`; correção depende de a thread principal ficar bloqueada durante `gen_steps`. Serialização do resultado é determinística.
4. **Acoplamento ao reactor/greenlet:** `toolhead`, `motion_queuing`, `homing`, `manual_stepper` e `idex_modes` chamam `reactor.pause`, usam `Completion` e timers. A decisão de modelo assíncrono do Rust (corrotinas, threads, loop de eventos) vira dependência de todo o recorte.
5. **Dinamismo Python:** `importlib` para `kinematics.<nome>`, `getattr(ffi_lib, alloc_func)`, `getattr(self, 'cmd_' + cmd)` em `gcode_move`, regex de shapers (`mzv(5,0.6)`), parser de `carriages:`. Todo isso vira tabelas explícitas no Rust.
6. **Fronteira com módulos de outros recortes:** `mcu.py` (`MCU_endstop`, `MCU_trsync`, clocksync, `print_time_to_clock`, `calibrate_clock`), `serialhdl.py`, `heaters`, `probe`, `bulk_sensor`, `webhooks`, `configfile`. `homing` e `stepper` não podem ser fechados sem esses contratos.
7. **Instrumentação do fork** misturada ao código de produção (C e Python). Decidir se entra no port (ver 7.8). Ela adiciona ~100 B/passo e `monotonic()` por lote; para paridade de bytes não importa.
8. **Dependência de linha de log:** `stats()` do toolhead e o string de `serialqueue_get_stats`; confirmar com o recorte de `statistics`/`scripts/` quem os lê.
9. **Modo ao vivo é não determinístico** (relógio real); só o modo arquivo permite diff de bytes.
10. **Lacunas desta análise:** `trapq.c`, `steppersync.c`, `msgblock.c`, `pollreactor.c`, `trdispatch.c`, `kin_*.c` (exceto trechos de `kin_shaper.c`), `stepper.py`, `kinematics/*`, `gcode_move.py`, `homing.py`, `force_move.py`, `manual_stepper.py`, `motion_report.py` não foram comparados linha a linha com o upstream; li os fontes e não achei marcadores do fork. Rodar `git diff <base-upstream> HEAD -- <arquivos>` fecha isso.

### Ordem de port (folhas primeiro)
1. `list.h`/estruturas de dados, `compiler.h` (trivial), `msgblock` (CRC16, VLQ, `clock_*`), `pollreactor`, `pyhelper` (log + monotonic RAW).
2. Fórmulas puras: `mathutil` (trilateration, matrizes, gaussian), `shaper_defs` (A/T), cálculos de `Move` e `LookAheadQueue` (testáveis isolados com golden de `trapq_append`).
3. Bloco A em C+Rust misto ou completo: `trapq`, `kin_cartesian/corexy/corexz/generic/winch/delta/deltesian`, depois `kin_polar/rotary_delta` (libm), `kin_idex`, `kin_shaper`, `kin_extruder`, e por fim `itersolve` (consome tudo isso).
4. Bloco B: `stepcompress` (maior risco numérico em inteiros) com fuzz diferencial contra o C; `steppersync`; `serialqueue` + `trdispatch`.
5. Camada de orquestração: `stepper.py`, `kinematics/*.py` (cartesian primeiro, os outros por demanda), `extruder.py`, `motion_queuing`, `toolhead`, `gcode_move`, `stepper_enable`, `force_move`, `manual_stepper`, `input_shaper`, `homing` (por último; depende de `mcu`), `motion_report`, `idex_modes`/`generic_cartesian` (últimos; maior superfície de config).
6. Para a KE o caminho mínimo é: itens 1 e 2, `trapq`, `kin_cartesian`, `kin_extruder`, `kin_shaper`, `itersolve`, `stepcompress`, `steppersync`, `serialqueue` (+ uart `/dev/ttyS1` 230400 e o MCU host `rpi`), `toolhead`, `motion_queuing`, `stepper`, `cartesian.py`, `extruder.py`, `input_shaper`, `gcode_move`, `homing` (com `bltouch`/`z_virtual_endstop`), `motion_report`, `stepper_enable`, `force_move`.
