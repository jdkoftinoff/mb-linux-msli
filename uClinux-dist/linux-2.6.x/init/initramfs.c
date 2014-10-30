#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/dirent.h>
#include <linux/syscalls.h>
#include <linux/utime.h>
#include <linux/kernel.h>

static __initdata char *message;
static void __init error(char *x)
{
	if (!message)
		message = x;
}

/* link hash */

#define N_ALIGN(len) ((((len) + 1) & ~3) + 2)

static __initdata struct hash {
	int ino, minor, major;
	mode_t mode;
	struct hash *next;
	char name[N_ALIGN(PATH_MAX)];
} *head[32];

static inline int hash(int major, int minor, int ino)
{
	unsigned long tmp = ino + minor + (major << 3);
	tmp += tmp >> 5;
	return tmp & 31;
}

static char __init *find_link(int major, int minor, int ino,
			      mode_t mode, char *name)
{
	struct hash **p, *q;
	for (p = head + hash(major, minor, ino); *p; p = &(*p)->next) {
		if ((*p)->ino != ino)
			continue;
		if ((*p)->minor != minor)
			continue;
		if ((*p)->major != major)
			continue;
		if (((*p)->mode ^ mode) & S_IFMT)
			continue;
		return (*p)->name;
	}
	q = kmalloc(sizeof(struct hash), GFP_KERNEL);
	if (!q)
		panic("can't allocate link hash entry");
	q->major = major;
	q->minor = minor;
	q->ino = ino;
	q->mode = mode;
	strcpy(q->name, name);
	q->next = NULL;
	*p = q;
	return NULL;
}

static void __init free_hash(void)
{
	struct hash **p, *q;
	for (p = head; p < head + 32; p++) {
		while (*p) {
			q = *p;
			*p = q->next;
			kfree(q);
		}
	}
}

static long __init do_utime(char __user *filename, time_t mtime)
{
	struct timespec t[2];

	t[0].tv_sec = mtime;
	t[0].tv_nsec = 0;
	t[1].tv_sec = mtime;
	t[1].tv_nsec = 0;

	return do_utimes(AT_FDCWD, filename, t, AT_SYMLINK_NOFOLLOW);
}

static __initdata LIST_HEAD(dir_list);
struct dir_entry {
	struct list_head list;
	char *name;
	time_t mtime;
};

static void __init dir_add(const char *name, time_t mtime)
{
	struct dir_entry *de = kmalloc(sizeof(struct dir_entry), GFP_KERNEL);
	if (!de)
		panic("can't allocate dir_entry buffer");
	INIT_LIST_HEAD(&de->list);
	de->name = kstrdup(name, GFP_KERNEL);
	de->mtime = mtime;
	list_add(&de->list, &dir_list);
}

static void __init dir_utime(void)
{
	struct dir_entry *de, *tmp;
	list_for_each_entry_safe(de, tmp, &dir_list, list) {
		list_del(&de->list);
		do_utime(de->name, de->mtime);
		kfree(de->name);
		kfree(de);
	}
}

static __initdata time_t mtime;

/* cpio header parsing */

static __initdata unsigned long ino, major, minor, nlink;
static __initdata mode_t mode;
static __initdata unsigned long body_len, name_len;
static __initdata uid_t uid;
static __initdata gid_t gid;
static __initdata unsigned rdev;

static void __init parse_header(char *s)
{
	unsigned long parsed[12];
	char buf[9];
	int i;

	buf[8] = '\0';
	for (i = 0, s += 6; i < 12; i++, s += 8) {
		memcpy(buf, s, 8);
		parsed[i] = simple_strtoul(buf, NULL, 16);
	}
	ino = parsed[0];
	mode = parsed[1];
	uid = parsed[2];
	gid = parsed[3];
	nlink = parsed[4];
	mtime = parsed[5];
	body_len = parsed[6];
	major = parsed[7];
	minor = parsed[8];
	rdev = new_encode_dev(MKDEV(parsed[9], parsed[10]));
	name_len = parsed[11];
}

/* FSM */

static __initdata enum state {
	Start,
	Collect,
	GotHeader,
	SkipIt,
	GotName,
	CopyFile,
	GotSymlink,
	Reset
} state, next_state;

static __initdata char *victim;
static __initdata unsigned count;
static __initdata loff_t this_header, next_header;

static inline void __init eat(unsigned n)
{
	victim += n;
	this_header += n;
	count -= n;
}

static __initdata char *vcollected;
static __initdata char *collected;
static __initdata int remains;
static __initdata char *collect;

static void __init read_into(char *buf, unsigned size, enum state next)
{
	if (count >= size) {
		collected = victim;
		eat(size);
		state = next;
	} else {
		collect = collected = buf;
		remains = size;
		next_state = next;
		state = Collect;
	}
}

static __initdata char *header_buf, *symlink_buf, *name_buf;

static int __init do_start(void)
{
	read_into(header_buf, 110, GotHeader);
	return 0;
}

static int __init do_collect(void)
{
	unsigned n = remains;
	if (count < n)
		n = count;
	memcpy(collect, victim, n);
	eat(n);
	collect += n;
	if ((remains -= n) != 0)
		return 1;
	state = next_state;
	return 0;
}

static int __init do_header(void)
{
	if (memcmp(collected, "070707", 6)==0) {
		error("incorrect cpio method used: use -H newc option");
		return 1;
	}
	if (memcmp(collected, "070701", 6)) {
		error("no cpio magic");
		return 1;
	}
	parse_header(collected);
	next_header = this_header + N_ALIGN(name_len) + body_len;
	next_header = (next_header + 3) & ~3;
	state = SkipIt;
	if (name_len <= 0 || name_len > PATH_MAX)
		return 0;
	if (S_ISLNK(mode)) {
		if (body_len > PATH_MAX)
			return 0;
		collect = collected = symlink_buf;
		remains = N_ALIGN(name_len) + body_len;
		next_state = GotSymlink;
		state = Collect;
		return 0;
	}
	if (S_ISREG(mode) || !body_len)
		read_into(name_buf, N_ALIGN(name_len), GotName);
	return 0;
}

static int __init do_skip(void)
{
	if (this_header + count < next_header) {
		eat(count);
		return 1;
	} else {
		eat(next_header - this_header);
		state = next_state;
		return 0;
	}
}

static int __init do_reset(void)
{
	while(count && *victim == '\0')
		eat(1);
	if (count && (this_header & 3))
		error("broken padding");
	return 1;
}

static int __init maybe_link(void)
{
	if (nlink >= 2) {
		char *old = find_link(major, minor, ino, mode, collected);
		if (old)
			return (sys_link(old, collected) < 0) ? -1 : 1;
	}
	return 0;
}

static void __init clean_path(char *path, mode_t mode)
{
	struct stat st;

	if (!sys_newlstat(path, &st) && (st.st_mode^mode) & S_IFMT) {
		if (S_ISDIR(st.st_mode))
			sys_rmdir(path);
		else
			sys_unlink(path);
	}
}

static __initdata int wfd;

int handle_special_name(void) {
    /* "./dev/@mtdblock5,b,31,5", */
    char * ptr;
    int parsed_major;
    int parsed_minor;
    int parsed_mode;
    char * start;
    char * end;
    ptr=strchr(collected,'@');
    if(ptr!=NULL) {
        start=ptr;
        ptr=strchr(collected,',');
        if(ptr!=NULL) {
            end=ptr;
            ptr++;
            if(ptr&&*ptr=='c') {
                parsed_mode=S_IFCHR;
            } else if(ptr&&*ptr=='b') {
                parsed_mode=S_IFBLK;
            } else if(ptr&&*ptr=='p') {
                parsed_mode=S_IFIFO;
            } else if(ptr&&*ptr=='s') {
                parsed_mode=S_IFSOCK;
            } else {
                return(0);
            }
            ptr++;
            if(*ptr++!=',') {
                return(0);
            }
            if(ptr) {
                parsed_major=simple_strtol(ptr,&ptr,10);
            } else {
                return(0);
            }
            if(ptr&&*ptr++==',') {
                parsed_minor=simple_strtol(ptr,&ptr,10);
                if(*ptr!='\0') {
                    return(0);
                } else {
                    *end='\0';
                    mode=parsed_mode;
                    rdev=new_encode_dev(MKDEV(parsed_major, parsed_minor));
                    while(*start!='\0') {
                        *start=*(start+1);
                        start++;
                    }
                    return(1);
                }
            } else {
                return(0);
            }
        }
    }
    return(0);;
}

static int __init do_name(void)
{
	state = SkipIt;
	next_state = Reset;
	if (strcmp(collected, "TRAILER!!!") == 0) {
		free_hash();
		return 0;
	}
	clean_path(collected, mode);
	handle_special_name();
	if(gid>=1000) {
		gid=0;
	}
	if(uid>=1000) {
		uid=0;
	}
	if (S_ISREG(mode)) {
		int ml = maybe_link();
		if (ml >= 0) {
			int openflags = O_WRONLY|O_CREAT;
			if (ml != 1)
				openflags |= O_TRUNC;
			wfd = sys_open(collected, openflags, mode);

			if (wfd >= 0) {
				sys_fchown(wfd, uid, gid);
				sys_fchmod(wfd, mode);
				if (body_len)
					sys_ftruncate(wfd, body_len);
				vcollected = kstrdup(collected, GFP_KERNEL);
				state = CopyFile;
			}
		}
	} else if (S_ISDIR(mode)) {
		sys_mkdir(collected, mode);
		sys_chown(collected, uid, gid);
		sys_chmod(collected, mode);
		dir_add(collected, mtime);
	} else if (S_ISBLK(mode) || S_ISCHR(mode) ||
		   S_ISFIFO(mode) || S_ISSOCK(mode)) {
		if (maybe_link() == 0) {
			sys_mknod(collected, mode, rdev);
			sys_chown(collected, uid, gid);
			sys_chmod(collected, mode);
			do_utime(collected, mtime);
		}
	}
	return 0;
}

static int __init do_copy(void)
{
	if (count >= body_len) {
		sys_write(wfd, victim, body_len);
		sys_close(wfd);
		do_utime(vcollected, mtime);
		kfree(vcollected);
		eat(body_len);
		state = SkipIt;
		return 0;
	} else {
		sys_write(wfd, victim, count);
		body_len -= count;
		eat(count);
		return 1;
	}
}

static int __init do_symlink(void)
{
	collected[N_ALIGN(name_len) + body_len] = '\0';
	clean_path(collected, 0);
	sys_symlink(collected + N_ALIGN(name_len), collected);
	sys_lchown(collected, uid, gid);
	do_utime(collected, mtime);
	state = SkipIt;
	next_state = Reset;
	return 0;
}

static __initdata int (*actions[])(void) = {
	[Start]		= do_start,
	[Collect]	= do_collect,
	[GotHeader]	= do_header,
	[SkipIt]	= do_skip,
	[GotName]	= do_name,
	[CopyFile]	= do_copy,
	[GotSymlink]	= do_symlink,
	[Reset]		= do_reset,
};

static int __init write_buffer(char *buf, unsigned len)
{
	count = len;
	victim = buf;

	while (!actions[state]())
		;
	return len - count;
}

static int __init flush_buffer(void *bufv, unsigned len)
{
	char *buf = (char *) bufv;
	int written;
	int origLen = len;
	if (message)
		return -1;
	while ((written = write_buffer(buf, len)) < len && !message) {
		char c = buf[written];
		if (c == '0') {
			buf += written;
			len -= written;
			state = Start;
		} else if (c == 0) {
			buf += written;
			len -= written;
			state = Reset;
		} else
			error("junk in compressed archive");
	}
	return origLen;
}

static unsigned my_inptr;   /* index of next byte to be processed in inbuf */

static __initdata int rfd;

static int __init fill(void *dst, unsigned len)
{
	int r = sys_read(rfd,dst,len);
	if (r < 0)
		printk(KERN_ERR "JIFFY: error while reading compressed data");
	else if (r == 0)
		printk(KERN_ERR "JIFFY: EOF while reading compressed data");
	return r;
}

#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
typedef enum {
  PROGRESS_COLOR_GREEN,
  PROGRESS_COLOR_RED,
  PROGRESS_COLOR_YELLOW
} progress_color_t;

static unsigned char bar_offsets[]=
  {1,4,6,9,11,17,19,30,32,38,40,46,48,54,56,62};

static int __init rd_display_progress(int fd, int n,progress_color_t color)
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

//  if(color==PROGRESS_COLOR_GREEN)
//    {
//      sys_write(fd,scroll_command,sizeof(scroll_command));
//    }
  return 0;
}

static int __init rd_create_char_dev(char *name, dev_t dev)
{
  sys_unlink(name);
  return sys_mknod(name, S_IFCHR|0600, new_encode_dev(dev));
}
#endif

#include <linux/decompress/generic.h>

char * __init unpack_to_rootfs_from_dev(int start) {
#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
	int progress_fd;
#endif

	const unsigned char arr[2] = {037, 0213};
	//int written;
	decompress_fn decompress;
	const char *compress_name;
	static __initdata char msg_buf[64];

#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
	/* device to display progress indicator or messages */
	rd_create_char_dev("/dev/progress",
				MKDEV(CONFIG_RAMDISK_PROGRESS_DEVICE_MAJOR,
				CONFIG_RAMDISK_PROGRESS_DEVICE_MINOR));
	progress_fd=sys_open("/dev/progress",O_WRONLY,0);
#endif

	header_buf = kmalloc(110, GFP_KERNEL);
	symlink_buf = kmalloc(PATH_MAX + N_ALIGN(PATH_MAX) + 1, GFP_KERNEL);
	name_buf = kmalloc(N_ALIGN(PATH_MAX), GFP_KERNEL);

	if (!header_buf || !symlink_buf || !name_buf)
		panic("can't allocate buffers");

	sys_unlink("/dev/jiffy");
	sys_mknod("/dev/jiffy", S_IFBLK|0600, new_encode_dev(MKDEV(CONFIG_RAMDISK_ALT_BOOT_DEVICE_MAJOR, CONFIG_RAMDISK_ALT_BOOT_DEVICE_MINOR)));

	printk(KERN_ALERT "Opening ramfs image at %s\n","/dev/jiffy");
	rfd = sys_open("/dev/jiffy", O_RDONLY, 0);
	if (rfd < 0) {
		printk("failed to open /dev/jiffy\r\n");
		strcpy(msg_buf,"failed to open /dev/jiffy");
#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
		rd_display_progress(progress_fd,4,PROGRESS_COLOR_RED);
#endif
		return(msg_buf);
	}

#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
	rd_display_progress(progress_fd,4,PROGRESS_COLOR_GREEN);
#endif

	if(sys_lseek(rfd, start * BLOCK_SIZE, 0)!=start * BLOCK_SIZE) {
		printk("failed seek to %d * %d /dev/jiffy\r\n",start,BLOCK_SIZE);
		strcpy(msg_buf,"failed seek /dev/jiffy\r\n");
		return(msg_buf);
	}

	state = Start;
	this_header = 0;
	message = NULL;
		this_header = 0;

	decompress = decompress_method(&arr[0], 2, &compress_name);
	if (decompress) {
		if(decompress(NULL, 0, fill , flush_buffer, NULL, &my_inptr, error)!=0) {
			printk("decompress failed\r\n");
#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
			rd_display_progress(progress_fd,5,PROGRESS_COLOR_RED);
#endif
		}
	}
	else if (compress_name) {
		if (!message) {
			snprintf(msg_buf, sizeof msg_buf,
				"compression method %s not configured",
				compress_name);
			message = msg_buf;
		}
	} else {
		printk("jiffy didn't run decompress\r\n");
#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
		rd_display_progress(progress_fd,5,PROGRESS_COLOR_RED);
#endif
	}
	if (state != Reset)
		error("junk in compressed archive");

#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
	rd_display_progress(progress_fd,5,PROGRESS_COLOR_GREEN);
#endif

#ifdef CONFIG_RAMDISK_DISPLAY_PROGRESS
    if(progress_fd>=0) {
        sys_close(progress_fd);
    }
    sys_unlink("/dev/progress");
#endif
	dir_utime();
	kfree(name_buf);
	kfree(symlink_buf);
	kfree(header_buf);
	sys_close(rfd);
	sys_unlink("/dev/jiffy");
	return message;
}

static char * __init unpack_to_rootfs(char *buf, unsigned len)
{
	int written;
	decompress_fn decompress;
	const char *compress_name;
	static __initdata char msg_buf[64];

	header_buf = kmalloc(110, GFP_KERNEL);
	symlink_buf = kmalloc(PATH_MAX + N_ALIGN(PATH_MAX) + 1, GFP_KERNEL);
	name_buf = kmalloc(N_ALIGN(PATH_MAX), GFP_KERNEL);

	if (!header_buf || !symlink_buf || !name_buf)
		panic("can't allocate buffers");

	state = Start;
	this_header = 0;
	message = NULL;
	while (!message && len) {
		loff_t saved_offset = this_header;
		if (*buf == '0' && !(this_header & 3)) {
			state = Start;
			written = write_buffer(buf, len);
			buf += written;
			len -= written;
			continue;
		}
		if (!*buf) {
			buf++;
			len--;
			this_header++;
			continue;
		}
		this_header = 0;
		decompress = decompress_method(buf, len, &compress_name);
		if (decompress)
			decompress(buf, len, NULL, flush_buffer, NULL,
				   &my_inptr, error);
		else if (compress_name) {
			if (!message) {
				snprintf(msg_buf, sizeof msg_buf,
					 "compression method %s not configured",
					 compress_name);
				message = msg_buf;
			}
		}
		if (state != Reset)
			error("junk in compressed archive");
		this_header = saved_offset + my_inptr;
		buf += my_inptr;
		len -= my_inptr;
	}
	dir_utime();
	kfree(name_buf);
	kfree(symlink_buf);
	kfree(header_buf);
	return message;
}

static int __initdata do_retain_initrd;

static int __init retain_initrd_param(char *str)
{
	if (*str)
		return 0;
	do_retain_initrd = 1;
	return 1;
}
__setup("retain_initrd", retain_initrd_param);

extern char __initramfs_start[], __initramfs_end[];
#include <linux/initrd.h>
#include <linux/kexec.h>

static void __init free_initrd(void)
{
#ifdef CONFIG_KEXEC
	unsigned long crashk_start = (unsigned long)__va(crashk_res.start);
	unsigned long crashk_end   = (unsigned long)__va(crashk_res.end);
#endif
	if (do_retain_initrd)
		goto skip;

#ifdef CONFIG_KEXEC
	/*
	 * If the initrd region is overlapped with crashkernel reserved region,
	 * free only memory that is not part of crashkernel region.
	 */
	if (initrd_start < crashk_end && initrd_end > crashk_start) {
		/*
		 * Initialize initrd memory region since the kexec boot does
		 * not do.
		 */
		memset((void *)initrd_start, 0, initrd_end - initrd_start);
		if (initrd_start < crashk_start)
			free_initrd_mem(initrd_start, crashk_start);
		if (initrd_end > crashk_end)
			free_initrd_mem(crashk_end, initrd_end);
	} else
#endif
		free_initrd_mem(initrd_start, initrd_end);
skip:
	initrd_start = 0;
	initrd_end = 0;
}

#ifdef CONFIG_BLK_DEV_RAM
#define BUF_SIZE 1024
static void __init clean_rootfs(void)
{
	int fd;
	void *buf;
	struct linux_dirent64 *dirp;
	int count;

	fd = sys_open("/", O_RDONLY, 0);
	WARN_ON(fd < 0);
	if (fd < 0)
		return;
	buf = kzalloc(BUF_SIZE, GFP_KERNEL);
	WARN_ON(!buf);
	if (!buf) {
		sys_close(fd);
		return;
	}

	dirp = buf;
	count = sys_getdents64(fd, dirp, BUF_SIZE);
	while (count > 0) {
		while (count > 0) {
			struct stat st;
			int ret;

			ret = sys_newlstat(dirp->d_name, &st);
			WARN_ON_ONCE(ret);
			if (!ret) {
				if (S_ISDIR(st.st_mode))
					sys_rmdir(dirp->d_name);
				else
					sys_unlink(dirp->d_name);
			}

			count -= dirp->d_reclen;
			dirp = (void *)dirp + dirp->d_reclen;
		}
		dirp = buf;
		memset(buf, 0, BUF_SIZE);
		count = sys_getdents64(fd, dirp, BUF_SIZE);
	}

	sys_close(fd);
	kfree(buf);
}
#endif

static int __init populate_rootfs(void)
{
	char *err = unpack_to_rootfs(__initramfs_start,
			 __initramfs_end - __initramfs_start);
	if (err)
		panic(err);	/* Failed to decompress INTERNAL initramfs */
	if (initrd_start) {
#ifdef CONFIG_BLK_DEV_RAM
		int fd;
		printk(KERN_INFO "Trying to unpack rootfs image as initramfs...\n");
		err = unpack_to_rootfs((char *)initrd_start,
			initrd_end - initrd_start);
		if (!err) {
			free_initrd();
			return 0;
		} else {
			clean_rootfs();
			unpack_to_rootfs(__initramfs_start,
				 __initramfs_end - __initramfs_start);
		}
		printk(KERN_INFO "rootfs image is not initramfs (%s)"
				"; looks like an initrd\n", err);
		fd = sys_open("/initrd.image", O_WRONLY|O_CREAT, 0700);
		if (fd >= 0) {
			sys_write(fd, (char *)initrd_start,
					initrd_end - initrd_start);
			sys_close(fd);
			free_initrd();
		}
#else
		printk(KERN_INFO "Unpacking initramfs...\n");
		err = unpack_to_rootfs((char *)initrd_start,
			initrd_end - initrd_start);
		if (err)
			printk(KERN_EMERG "Initramfs unpacking failed: %s\n", err);
		free_initrd();
#endif
	}
	return 0;
}
rootfs_initcall(populate_rootfs);
