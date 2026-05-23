extern int main(int argc, char **argv);
extern int __libc_start_main(int (*main)(int, char **, char **), int argc,
                             char **argv, void (*init)(void),
                             void (*fini)(void), void (*rtld_fini)(void),
                             void *stack_end);

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

__asm__(
".globl _start\n"
".type _start, @function\n"
"_start:\n"
"    move $4, $29\n"
"    li $8, -8\n"
"    and $29, $29, $8\n"
"    jal _start_c\n"
"    nop\n");
