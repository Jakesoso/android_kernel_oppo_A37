#ifdef VENDOR_EDIT
#fangpan@Swdp.shanghai, 2016/04/28, fix the tlb flush too long issue
obj-y				:= dma-mapping.o extable.o fault.o init.o \
				   cache.o copypage.o flush.o \
				   ioremap.o mmap.o pgd.o mmu.o \
				   context.o proc.o
#else
#obj-y				:= dma-mapping.o extable.o fault.o init.o \
#				   cache.o copypage.o flush.o \
#				   ioremap.o mmap.o pgd.o mmu.o \
#				   context.o tlb.o proc.o
#endif
