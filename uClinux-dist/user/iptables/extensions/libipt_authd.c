/* Shared library add-on to iptables for authd. */
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <xtables.h>

/* Our less than helpful help
 */
static void authd_help(void)
{
	printf("authd match takes no options\n");
}

/* Initialize ourselves.
 */
static void authd_init(struct xt_entry_match *m)
{
}

/* Parse command options.
 * Since we have no options we never consume any and thus always
 * return false.
 */
static int authd_parse(int c, char **argv, int invert, unsigned int *flags,
		const void *entry, struct xt_entry_match **match)
{
	return 0;
}

/* Globals that contain our information.
 * We take no options so that structure is empty.
 */
static struct option authd_opts[] = {
	{ .name = NULL }
};

/* All of our information and work functions live here.
 */
static struct xtables_match authd_match = {
	.name		= "authd",
	.version	= XTABLES_VERSION,
	.family		= NFPROTO_IPV4,
	.size		= XT_ALIGN(0),
	.userspacesize	= XT_ALIGN(0),
	.help		= &authd_help,
	.init		= &authd_init,
	.parse		= &authd_parse,
	.extra_opts	= authd_opts,
};

/* Our initialisation code.  Just register us as a target and that's it.
 * The kernel module and the user land authd process will take care of
 * everything.
 */
void _init(void)
{
	xtables_register_match(&authd_match);
}
