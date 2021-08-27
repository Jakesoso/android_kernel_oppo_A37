#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/string.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/memory.h>

#include <linux/log_store_kernel.h>

#define FILE_PATH	"/data/pl_lk"



static struct sram_log_header *sram_header;
static int sram_log_store_status = BUFF_NOT_READY;
static int dram_log_store_status = BUFF_NOT_READY;
static char *pbuff;
static struct pl_lk_log *log_buff_header;
static struct dram_buf_header *sram_dram_buff;
static bool early_log_disable;

/* set the flag whether store log to emmc in next boot phase in pl */
void store_log_to_emmc_enable(bool value)
{
	if (sram_dram_buff == NULL) {
		pr_err("%s: sram dram buff is NULL.\n", __func__);
		return;
	}

	if (value)
		sram_dram_buff->flag |= NEED_SAVE_TO_EMMC;
	else
		sram_dram_buff->flag &= ~NEED_SAVE_TO_EMMC;

	pr_notice("log_store: sram_dram_buff flag 0x%x.\n", sram_dram_buff->flag);

	return;
}

void log_store_bootup(void)
{
	struct file *filp;
	int value;
	mm_segment_t fs;
	loff_t pos;

	/* Boot up finish, don't save log to emmc in next boot. */
	store_log_to_emmc_enable(false);

	if (log_buff_header == NULL || pbuff == NULL) {
		pr_err("%s: log buff is NULL.\n", __func__);
		//goto free_memory;
	}
#if 0

/* store log to /data/pl_lk file */
	filp = filp_open(FILE_PATH, O_CREAT|O_WRONLY|O_TRUNC, 0);
	value = (int)IS_ERR(filp);
	if (value) {
		pr_err("%s: create file %s error 0x%x.\n", __func__, FILE_PATH, value);
		goto free_memory;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(filp, pbuff+log_buff_header->off_pl,
		log_buff_header->sz_lk + log_buff_header->sz_pl, &pos);
	filp_close(filp, NULL);
	set_fs(fs);

/*  Free the reserve memory */
free_memory:
	if (pbuff != NULL) {
		vunmap(pbuff);
		log_buff_header = NULL;
		pbuff = NULL;
	}
#endif
	return;
}

static void *remap_lowmem(phys_addr_t start, phys_addr_t size)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	prot = pgprot_noncached(PAGE_KERNEL);

	pages = kmalloc(sizeof(struct page *) * page_count, GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n", __func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);
	if (!vaddr) {
		pr_err("%s: Failed to map %u pages\n", __func__, page_count);
		return NULL;
	}

	return vaddr + offset_in_page(start);
}

static int pl_lk_log_show(struct seq_file *m, void *v)
{
	int i = 0;
	if (log_buff_header == NULL || pbuff == NULL) {
		seq_printf(m, "log buff is null.\n");
		return 0;
	}

	seq_printf(m, "show buff sig 0x%x, size 0x%x,pl size 0x%x, lk size 0x%x!\n", log_buff_header->sig,
			log_buff_header->buff_size, log_buff_header->sz_pl, log_buff_header->sz_lk);

	if (dram_log_store_status == BUFF_READY && log_buff_header->buff_size >= (log_buff_header->off_pl + log_buff_header->sz_pl + log_buff_header->sz_lk))
		seq_write(m, pbuff+log_buff_header->off_pl, log_buff_header->sz_lk + log_buff_header->sz_pl);

	return 0;
}


static int pl_lk_file_open(struct inode *inode, struct file *file)
{
	return single_open(file, pl_lk_log_show, inode->i_private);
}

static const struct file_operations pl_lk_file_ops = {
	.owner = THIS_MODULE,
	.open = pl_lk_file_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int __init log_store_late_init(void)
{
	struct proc_dir_entry *entry;

	if (sram_dram_buff == NULL)	{
		pr_err("log_store: sram header DRAM buff is null.\n");
		dram_log_store_status = BUFF_ALLOC_ERROR;
		return -1;
	}

	if (sram_dram_buff->buf_addr == 0 || sram_dram_buff->buf_size == 0)	{
		pr_err("log_store: sram header DRAM buff is null.\n");
		dram_log_store_status = BUFF_ALLOC_ERROR;
		return -1;
	}

	pr_notice("log store:sram_dram_buff addr 0x%x, size 0x%x.\n",
		sram_dram_buff->buf_addr, sram_dram_buff->buf_size);

	pbuff = remap_lowmem(sram_dram_buff->buf_addr, sram_dram_buff->buf_size);
	if (pbuff == NULL) {
		pr_err("log_store: ioremap failed.\n");
		dram_log_store_status = BUFF_ERROR;
		return -1;
	}

	log_buff_header = (struct pl_lk_log *)pbuff;

/*/ check buff flag */
	if (log_buff_header->sig != LOG_STORE_SIG) {
		pr_err("log store: log sig error:0x%x.\n", log_buff_header->sig);
		dram_log_store_status = BUFF_ERROR;
		return 0;
	}

	dram_log_store_status = BUFF_READY;
	pr_notice("log_store: log buff 0x%p, sig 0x%x, buff_size 0x%x, pl off 0x%x, pl sz 0x%x,lk off 0x%x, lk size 0x%x,flag 0x%x, 0x%x.\n",
		pbuff, log_buff_header->sig, log_buff_header->buff_size, log_buff_header->off_pl, log_buff_header->sz_pl,
		log_buff_header->off_lk, log_buff_header->sz_lk, log_buff_header->pl_flag, log_buff_header->lk_flag);

	entry = proc_create("pl_lk", 0444, NULL, &pl_lk_file_ops);
	if (!entry) {
		pr_err("log_store: failed to create proc entry\n");
		return 1;
	}

	return 0;
}

/* need mapping virtual address to phy address */
static void store_printk_buff(void)
{
	phys_addr_t log_buf;
	int size;

	if (sram_dram_buff == NULL)	{
		pr_err("log_store: sram_dram_buff is null.\n");
		return;
	}

	get_logbuf_info(&log_buf, &size);
	sram_dram_buff->reserve2[0] = (u32)log_buf;
	sram_dram_buff->reserve2[1] = size;
	if (early_log_disable == false)
		sram_dram_buff->flag |= BUFF_EARLY_PRINTK;

	pr_notice("log_store printk log buff add:0x%x, size 0x%x. buff flag 0x%x.\n",
		sram_dram_buff->reserve2[0], sram_dram_buff->reserve2[1], sram_dram_buff->flag);

	return;
}

void disable_early_log(void)
{
	pr_notice("log_store: disable_early_log.\n");
	early_log_disable = true;
	if (sram_dram_buff == NULL)	{
		pr_err("log_store: sram_dram_buff is null.\n");
		return;
	}

	sram_dram_buff->flag &= ~BUFF_EARLY_PRINTK;
	return;
}


/*  store log_store information to */
static int __init log_store_early_init(void)
{

	sram_header = ioremap(CONFIG_MTK_DRAM_LOG_STORE_ADDR, CONFIG_MTK_RAM_CONSOLE_SIZE);


	pr_err("log_store: sram header address 0x%p.\n", sram_header);
	if (sram_header->sig != SRAM_HEADER_SIG) {
		pr_err("log_store: sram header sig error 0x%x.\n", sram_header->sig);
		sram_log_store_status = BUFF_ERROR;
		sram_header = NULL;
		return -1;
	}

	sram_dram_buff = &(sram_header->dram_buf[LOG_PL_LK]);
	if (sram_dram_buff->sig != DRAM_HEADER_SIG)	{
		pr_err("log_store: sram header DRAM sig errorn");
		sram_log_store_status = BUFF_ERROR;
		sram_dram_buff = NULL;
		return -1;

	}

	/*  store printk log buff information to DRAM */
	store_printk_buff();

	 pr_notice("sram_dram_buff sig 0x%x, flag 0x%x, add 0x%x, size 0x%x, offsize 0x%x, point 0x%x.\n",
		sram_dram_buff->sig, sram_dram_buff->flag, sram_dram_buff->buf_addr,
		sram_dram_buff->buf_size, sram_dram_buff->buf_offsize, sram_dram_buff->buf_point);
	return 0;
}

early_initcall(log_store_early_init);
late_initcall(log_store_late_init);
