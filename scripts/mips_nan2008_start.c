extern int main(int argc, char **argv);
extern int __libc_start_main(int (*main)(int, char **, char **), int argc,
                             char **argv, void (*init)(void),
                             void (*fini)(void), void (*rtld_fini)(void),
                             void *stack_end);

/* crt1.o normally defines this. glibc's libio checks whether the program
   defines it; without it libc assumes a glibc 2.0 binary and switches
   stdin/stdout/stderr to the old, shorter FILE structs, and exit-time
   `_IO_cleanup` then reads `_mode`/`_wide_data` past their end and frees
   a garbage pointer (SIGBUS under qemu). Value: `_G_IO_IO_FILE_VERSION`. */
__attribute__((used))
const int _IO_stdin_used = 0x20001;

__attribute__((used, externally_visible, noinline))
void
_start_c(long *sp)
{
    int argc = (int)sp[0];
    char **argv = (char **)(sp + 1);
    __libc_start_main((int (*)(int, char **, char **))main, argc, argv,
                      0, 0, 0, sp);
    __builtin_unreachable();
}

/* $gp is not set up yet at entry (it still holds whatever the dynamic linker
   left), so the call must not go through the GOT: in abicalls mode the
   assembler expands a plain `jal` into `lw $t9, %call16(_start_c)($gp)`,
   which faults on hardware. Load the absolute address (the executable is
   non-PIE) into $t9, which _start_c may also use to derive its own $gp. */
__asm__(
".globl _start\n"
".type _start, @function\n"
".set push\n"
".set noreorder\n"
".set nomacro\n"
"_start:\n"
"    move $4, $29\n"
"    li $8, -8\n"
"    and $29, $29, $8\n"
"    lui $25, %hi(_start_c)\n"
"    addiu $25, $25, %lo(_start_c)\n"
"    jalr $25\n"
"    nop\n"
".set pop\n");
