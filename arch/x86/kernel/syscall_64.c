/* System call table for x86-64. */

#include <linux/linkage.h>
#include <linux/sys.h>
#include <linux/cache.h>
#include <asm/asm-offsets.h>

#define __SYSCALL_COMMON(nr, sym, qual) __SYSCALL_64(nr, sym)

#ifdef CONFIG_X86_X32_ABI
# define __SYSCALL_X32(nr, sym), qual __SYSCALL_64(nr, sym, qual)
#else
# define __SYSCALL_X32(nr, sym, qual) /* nothing */
#endif


#define __SYSCALL_64_QUAL_(sym) sym
#define __SYSCALL_64_QUAL_ptregs(sym) ptregs_##sym
#define __SYSCALL_64(nr, sym, qual) extern asmlinkage void __SYSCALL_64_QUAL_##qual(sym)(void) ;
#include <asm/syscalls_64.h>
#undef __SYSCALL_64

#define __SYSCALL_64(nr, sym, qual) [nr] =  __SYSCALL_64_QUAL_##qual(sym),

typedef void (*sys_call_ptr_t)(void);

extern void sys_ni_syscall(void);

const sys_call_ptr_t sys_call_table[__NR_syscall_max+1] = {
	/*
	 * Smells like a compiler bug -- it doesn't work
	 * when the & below is removed.
	 */
	[0 ... __NR_syscall_max] = &sys_ni_syscall,
#include <asm/syscalls_64.h>
};
