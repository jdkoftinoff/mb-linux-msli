
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/minix_fs.h>
#include <linux/ext2_fs.h>
#include <linux/romfs_fs.h>
#include <linux/cramfs_fs.h>
#include <linux/initrd.h>
#include <linux/string.h>

#include "do_mounts.h"
#include "../fs/squashfs/squashfs_fs.h"

#include <linux/decompress/generic.h>


static int rd_failed=0,rd_start_flag=1;
static char *rd_start, *rd_end, *rd_ptr;

int rd_open(const char *path,int flags,mode_t mode)
{
  int fd;
  fd=sys_open(path,flags,mode);

  if(fd<0){
    if(rd_start_flag){
      /* no ramdisk device, last-resort attempt -- image may be preloaded
         at this fixed address */
      rd_start=ioremap(0x97c00000,0x400000);
      rd_end=rd_start+0x400000;
      rd_ptr=rd_start;
      rd_start_flag=0;
    }else{
      rd_failed=1;
    }
  }
  return fd;
}

size_t rd_read(int fd, void *buf, size_t len)
{
  unsigned long bufleft;
  if(fd>=0) {
    return sys_read(fd,buf,len);
  } else {
    bufleft=rd_end-rd_ptr;
    if(len>bufleft) len=bufleft;
    memcpy(buf,rd_ptr,len);
    rd_ptr+=len;
    return len;
  }
}

size_t rd_lseek(int fd, off_t offset, int whence)
{
  if(fd>=0) {
    return sys_lseek(fd,offset,whence);
  } else {
    switch(whence)
      {
      case SEEK_SET:
       rd_ptr=rd_start+offset;
       break;
      case SEEK_END:
       rd_ptr=rd_end+offset;
       break;
      case SEEK_CUR:
       rd_ptr+=offset;
       break;
      default:
       return -1;
      }
    if(rd_ptr<rd_start) rd_ptr=rd_start;
    if(rd_ptr>rd_end) rd_ptr=rd_end;
    return rd_ptr-rd_start;
  }
}

int rd_close(int fd){
  if(fd>=0) {
    return sys_close(fd);
  } else {
    iounmap(rd_start);
    return 0;
  }
}

int __initdata rd_prompt = 1;/* 1 = prompt for RAM disk, 0 = don't prompt */

static int __init prompt_ramdisk(char *str)
{
	rd_prompt = simple_strtol(str,NULL,0) & 1;
	return 1;
}
__setup("prompt_ramdisk=", prompt_ramdisk);

int __initdata rd_image_start;		/* starting block # of image */

static int __init ramdisk_start_setup(char *str)
{
	rd_image_start = simple_strtol(str,NULL,0);
	return 1;
}
__setup("ramdisk_start=", ramdisk_start_setup);

static int __init crd_load(int in_fd, int out_fd, decompress_fn deco);

/*
 * This routine tries to find a RAM disk image to load, and returns the
 * number of blocks to read for a non-compressed image, 0 if the image
 * is a compressed image, and -1 if an image with the right magic
 * numbers could not be found.
 *
 * We currently check for the following magic numbers:
 *	minix
 *	ext2
 *	romfs
 *	cramfs
 *	squashfs
 *	gzip
 */
static int __init
identify_ramdisk_image(int fd, int start_block, decompress_fn *decompressor)
{
	const int size = 512;
	struct minix_super_block *minixsb;
	struct ext2_super_block *ext2sb;
	struct romfs_super_block *romfsb;
	struct cramfs_super *cramfsb;
	struct squashfs_super_block *squashfsb;
	int nblocks = -1;
	unsigned char *buf;
	const char *compress_name;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return -1;

	minixsb = (struct minix_super_block *) buf;
	ext2sb = (struct ext2_super_block *) buf;
	romfsb = (struct romfs_super_block *) buf;
	cramfsb = (struct cramfs_super *) buf;
	squashfsb = (struct squashfs_super_block *) buf;
	memset(buf, 0xe5, size);

	/*
	 * Read block 0 to test for compressed kernel
	 */
	rd_lseek(fd, start_block * BLOCK_SIZE, 0);
	rd_read(fd, buf, size);

	*decompressor = decompress_method(buf, size, &compress_name);
	if (compress_name) {
		printk(KERN_NOTICE "RAMDISK: %s image found at block %d\n",
		       compress_name, start_block);
		if (!*decompressor)
			printk(KERN_EMERG
			       "RAMDISK: %s decompressor not configured!\n",
			       compress_name);
		nblocks = 0;
		goto done;
	}

	/* romfs is at block zero too */
	if (romfsb->word0 == ROMSB_WORD0 &&
	    romfsb->word1 == ROMSB_WORD1) {
		printk(KERN_NOTICE
		       "RAMDISK: romfs filesystem found at block %d\n",
		       start_block);
		nblocks = (ntohl(romfsb->size)+BLOCK_SIZE-1)>>BLOCK_SIZE_BITS;
		goto done;
	}

	if (cramfsb->magic == CRAMFS_MAGIC) {
		printk(KERN_NOTICE
		       "RAMDISK: cramfs filesystem found at block %d\n",
		       start_block);
		nblocks = (cramfsb->size + BLOCK_SIZE - 1) >> BLOCK_SIZE_BITS;
		goto done;
	}

	/* squashfs is at block zero too */
	if (le32_to_cpu(squashfsb->s_magic) == SQUASHFS_MAGIC) {
		printk(KERN_NOTICE
		       "RAMDISK: squashfs filesystem found at block %d\n",
		       start_block);
		nblocks = (le64_to_cpu(squashfsb->bytes_used) + BLOCK_SIZE - 1)
			 >> BLOCK_SIZE_BITS;
		goto done;
	}

	/*
	 * Read block 1 to test for minix and ext2 superblock
	 */
	rd_lseek(fd, (start_block+1) * BLOCK_SIZE, 0);
	rd_read(fd, buf, size);

	/* Try minix */
	if (minixsb->s_magic == MINIX_SUPER_MAGIC ||
	    minixsb->s_magic == MINIX_SUPER_MAGIC2) {
		printk(KERN_NOTICE
		       "RAMDISK: Minix filesystem found at block %d\n",
		       start_block);
		nblocks = minixsb->s_nzones << minixsb->s_log_zone_size;
		goto done;
	}

	/* Try ext2 */
	if (ext2sb->s_magic == cpu_to_le16(EXT2_SUPER_MAGIC)) {
		printk(KERN_NOTICE
		       "RAMDISK: ext2 filesystem found at block %d\n",
		       start_block);
		nblocks = le32_to_cpu(ext2sb->s_blocks_count) <<
			le32_to_cpu(ext2sb->s_log_block_size);
		goto done;
	}

	printk(KERN_NOTICE
	       "RAMDISK: Couldn't find valid RAM disk image starting at %d.\n",
	       start_block);

done:
	rd_lseek(fd, start_block * BLOCK_SIZE, 0);
	kfree(buf);
	return nblocks;
}

typedef enum {
  PROGRESS_COLOR_GREEN,
  PROGRESS_COLOR_RED,
  PROGRESS_COLOR_YELLOW
} progress_color_t;

static unsigned char bar_offsets[]=
  {1,4,6,9,11,17,19,30,32,38,40,46,48,54,56,62};

int __init rd_display_progress(int fd, int n,progress_color_t color)
{
  unsigned char command[]={
    /* 0     1     2 */
    0x2e, 0x26, 0x01, 
    /* 2     3     4     5     6 */
    0x22, 0x00, 0x00, 0x00, 0x00,
    /* 7     8     9    10    11    12 */
    0x00,  0x3f, 0x00, 0x00, 0x3f, 0x00,
   /* 13   14 */
    0x26, 0x01, 
   /* 15    16    17    18    19    20    21 */
    0x23, 0x00, 0x10, 0x0f, 0x27, 0x18, 0x0d
  };

  unsigned char error_command[]={
    0x2e, 0x26, 0x01, 
    0x22, 0x10, 0x00, 0x4f, 0x0b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  unsigned char scroll_command[]={
    0x27, 0x5e, 0x00, 0x0c, 0x00, 0x00, 0x2f
  };
  
  if(n<1) n=1;
  else if(n>8) n=8;

  command[4]=16+bar_offsets[n*2-2];
  command[5]=38;
  command[6]=16+bar_offsets[n*2-1];
  command[7]=43;
  
#if 0	
  command[8]=0xe0; /*b*/
  command[9]=0xe0; /*g*/
  command[10]=0xe0; /*r*/
  command[11]=0x40; /*b*/
  command[12]=0xf0; /*g*/
  command[13]=0x40; /*r*/
#endif

  if(color != PROGRESS_COLOR_GREEN)
    {
      sys_write(fd,error_command,sizeof(error_command));
      /* show error message */
      command[17]=0x10;
      command[18]=0x30;
      command[19]=0x4f;
      command[20]=0x3b;
      command[21]=0x10;
      command[22]=0x00;
    }
  else
    {
      /* update percentage */
      switch(n)
	{
	case 4:
	  /* "5" is drawn there, for "50%" */
	  command[18]=0x10;
	  command[20]=0x27;
	  break;
	case 5:
	  /* "6" is drawn there, for "60%" */
	  command[18]=0x28;
	  command[20]=0x3f;
	  break;
	}
    }
  
  switch(color)
    {
    case PROGRESS_COLOR_RED:
      command[8]=0x00; /*b*/
      command[9]=0x00; /*g*/
      command[10]=0x3e; /*r*/
      command[11]=0x00; /*b*/
      command[12]=0x00; /*g*/
      command[13]=0x3e; /*r*/
      break;

    case PROGRESS_COLOR_YELLOW:
      command[8]=0x00; /*b*/
      command[9]=0x3f; /*g*/
      command[10]=0x3e; /*r*/
      command[11]=0x00; /*b*/
      command[12]=0x3f; /*g*/
      command[13]=0x3e; /*r*/
      break;

    default:
      break;
    }
  
  sys_write(fd,command,sizeof(command));
  
  if(color==PROGRESS_COLOR_GREEN)
    {
      sys_write(fd,scroll_command,sizeof(scroll_command));
    }


  return 0;
}

int __init rd_create_char_dev(char *name, dev_t dev)
{
  sys_unlink(name);
  return sys_mknod(name, S_IFCHR|0600, new_encode_dev(dev));
}

int __init rd_load_image(char *from)
{
	int res = 0;
	int in_fd, out_fd, progress_fd;
	unsigned long rd_blocks, devblocks;
	int nblocks, i, disk;
	char *buf = NULL;
	unsigned short rotate = 0;
	decompress_fn decompressor = NULL;
#if !defined(CONFIG_S390) && !defined(CONFIG_PPC_ISERIES)
	char rotator[4] = { '|' , '/' , '-' , '\\' };
#endif

	/* device to display progress indicator or messages */
        rd_create_char_dev("/dev/progress",MKDEV(153,0));
	progress_fd=sys_open("/dev/progress",O_WRONLY,0);

	out_fd = sys_open("/dev/ram", O_RDWR, 0);
	if (out_fd < 0)
		goto out;

	printk(KERN_ALERT "Opening ramdisk image at %s\n",from);
	in_fd = rd_open(from, O_RDONLY, 0);
	if (in_fd < 0 && rd_failed) {
	  rd_display_progress(progress_fd,4,PROGRESS_COLOR_RED);
	  printk(KERN_ALERT "No ramdisk image\n");
		goto noclose_input;
	}

	rd_display_progress(progress_fd,4,PROGRESS_COLOR_GREEN);

	nblocks = identify_ramdisk_image(in_fd, in_fd>=0?rd_image_start:0, &decompressor);
	if (nblocks < 0)
		goto done;

	if (nblocks == 0) {
		if (crd_load(in_fd, out_fd, decompressor) == 0)
			goto successful_load;
		rd_display_progress(progress_fd,5,PROGRESS_COLOR_RED);
		goto done;
	}

	/*
	 * NOTE NOTE: nblocks is not actually blocks but
	 * the number of kibibytes of data to load into a ramdisk.
	 * So any ramdisk block size that is a multiple of 1KiB should
	 * work when the appropriate ramdisk_blocksize is specified
	 * on the command line.
	 *
	 * The default ramdisk_blocksize is 1KiB and it is generally
	 * silly to use anything else, so make sure to use 1KiB
	 * blocksize while generating ext2fs ramdisk-images.
	 */
	if (sys_ioctl(out_fd, BLKGETSIZE, (unsigned long)&rd_blocks) < 0)
		rd_blocks = 0;
	else
		rd_blocks >>= 1;

	if (nblocks > rd_blocks) {
		printk("RAMDISK: image too big! (%dKiB/%ldKiB)\n",
		       nblocks, rd_blocks);
		goto done;
	}

	/*
	 * OK, time to copy in the data
	 */
        if(in_fd<0)
                devblocks = 0;
        else
	if (sys_ioctl(in_fd, BLKGETSIZE, (unsigned long)&devblocks) < 0)
		devblocks = 0;
	else
		devblocks >>= 1;

	if (strcmp(from, "/initrd.image") == 0)
		devblocks = nblocks;

	if (devblocks == 0) {
		printk(KERN_ERR "RAMDISK: could not determine device size\n");
		goto done;
	}

	buf = kmalloc(BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		printk(KERN_ERR "RAMDISK: could not allocate buffer\n");
		goto done;
	}

	printk(KERN_NOTICE "RAMDISK: Loading %dKiB [%ld disk%s] into ram disk... ",
		nblocks, ((nblocks-1)/devblocks)+1, nblocks>devblocks ? "s" : "");
	for (i = 0, disk = 1; i < nblocks; i++) {
		if (i && (i % devblocks == 0)) {
			printk("done disk #%d.\n", disk++);
			rotate = 0;
			if (rd_close(in_fd)) {
				printk("Error closing the disk.\n");
				goto noclose_input;
			}
			change_floppy("disk #%d", disk);
			in_fd = rd_open(from, O_RDONLY, 0);
			if (in_fd < 0 && rd_failed)  {
				printk("Error opening disk.\n");
				goto noclose_input;
			}
			printk("Loading disk #%d... ", disk);
		}
		rd_read(in_fd, buf, BLOCK_SIZE);
		sys_write(out_fd, buf, BLOCK_SIZE);
#if !defined(CONFIG_S390) && !defined(CONFIG_PPC_ISERIES)
		if (!(i % 16)) {
			printk("%c\b", rotator[rotate & 0x3]);
			rotate++;
		}
#endif
	}
	printk("done.\n");

successful_load:
	res = 1;
	rd_display_progress(progress_fd,5,PROGRESS_COLOR_GREEN);
done:
	rd_close(in_fd);
noclose_input:
	sys_close(out_fd);
out:
	kfree(buf);
	if(progress_fd>=0) sys_close(progress_fd);
	sys_unlink("/dev/progress");
	sys_unlink("/dev/ram");
	return res;
}

int __init rd_load_disk(int n)
{
	if (rd_prompt)
		change_floppy("root floppy disk to be loaded into RAM disk");
        sys_unlink("/dev/root");
        create_dev("/dev/root",MKDEV(31,0) /* mtd */);
        /* create_dev("/dev/root",ROOT_DEV);*/
	create_dev("/dev/ram", MKDEV(RAMDISK_MAJOR, n));
	return rd_load_image("/dev/root");
}

static int exit_code;
static int decompress_error;
static int crd_infd, crd_outfd;

static int __init compr_fill(void *buf, unsigned int len)
{
	int r = rd_read(crd_infd, buf, len);
	if (r < 0)
		printk(KERN_ERR "RAMDISK: error while reading compressed data");
	else if (r == 0)
		printk(KERN_ERR "RAMDISK: EOF while reading compressed data");
	return r;
}

static int __init compr_flush(void *window, unsigned int outcnt)
{
	int written = sys_write(crd_outfd, window, outcnt);
	if (written != outcnt) {
		if (decompress_error == 0)
			printk(KERN_ERR
			       "RAMDISK: incomplete write (%d != %d)\n",
			       written, outcnt);
		decompress_error = 1;
		return -1;
	}
	return outcnt;
}

static void __init error(char *x)
{
	printk(KERN_ERR "%s\n", x);
	exit_code = 1;
	decompress_error = 1;
}

static int __init crd_load(int in_fd, int out_fd, decompress_fn deco)
{
	int result;
	crd_infd = in_fd;
	crd_outfd = out_fd;
	result = deco(NULL, 0, compr_fill, compr_flush, NULL, NULL, error);
	if (decompress_error)
		result = 1;
	return result;
}
