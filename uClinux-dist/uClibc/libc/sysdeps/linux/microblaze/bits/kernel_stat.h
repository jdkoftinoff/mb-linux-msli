/* Stat structure for linux/microblaze*/

#ifndef _BITS_STAT_STRUCT_H
#define _BITS_STAT_STRUCT_H

#ifndef _LIBC
#error bits/kernel_stat.h is for internal uClibc use only!
#endif

struct kernel_stat
{
	unsigned long	st_dev;
	unsigned long	st_ino;
	unsigned int	st_mode;
	unsigned int	st_nlink;
	unsigned int	st_uid;
	unsigned int	st_gid;
	unsigned long	st_rdev;
	unsigned long	__pad1;
	long		st_size;
	int		st_blksize;
	int		__pad2;
	long		st_blocks;
	int		st_atime;
	unsigned int	st_atime_nsec;
	int		st_mtime;
	unsigned int	st_mtime_nsec;
	int		st_ctime;
	unsigned int	st_ctime_nsec;
	unsigned long	__unused4;
	unsigned long	__unused5;
};

struct kernel_stat64
{
	unsigned long long	st_dev;		/* Device.  */
	unsigned long long	st_ino;		/* File serial number.  */
	unsigned int		st_mode;	/* File mode.  */
	unsigned int		st_nlink;	/* Link count.  */
	unsigned int		st_uid;		/* User ID of the file's owner.  */
	unsigned int		st_gid;		/* Group ID of the file's group. */
	unsigned long long	st_rdev;	/* Device number, if device.  */
	unsigned long long	__pad1;
	long long		st_size;	/* Size of file, in bytes.  */
	int			st_blksize;	/* Optimal block size for I/O.  */
	int			__pad2;
	long long		st_blocks;	/* Number 512-byte blocks allocated. */
	int			st_atime;	/* Time of last access.  */
	unsigned int		st_atime_nsec;
	int			st_mtime;	/* Time of last modification.  */
	unsigned int		st_mtime_nsec;
	int			st_ctime;	/* Time of last status change.  */
	unsigned int		st_ctime_nsec;
	unsigned int		__unused4;
	unsigned int		__unused5;
};

#endif	/*  _BITS_STAT_STRUCT_H */
