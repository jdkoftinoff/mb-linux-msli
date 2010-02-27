IPOBJ=ip.o ipaddress.o ipaddrlabel.o iproute.o iprule.o \
    rtm_map.o iptunnel.o tunnel.o ipneigh.o ipntable.o iplink.o \
    ipmaddr.o ipmonitor.o ipprefix.o
ifdef CONFIG_IPV6_TUNNEL
IPOBJ += ip6tunnel.o
endif
ifdef CONFIG_IP_MROUTE
IPOBJ += ipmroute.o
endif
ifdef CONFIG_XFRM
IPOBJ += ipxfrm.o xfrm_state.o xfrm_policy.o xfrm_monitor.o
endif

LINK_OPT =
ifdef CONFIG_VLAN_8021Q
LINK_OPT += vlan
IPOBJ += iplink_vlan.o
endif
ifdef CONFIG_VETH
LINK_OPT += veth
IPOBJ += link_veth.o
endif
ifdef CONFIG_NET_IPGRE
LINK_OPT += gre gretap
IPOBJ += link_gre.o
endif

ifneq ($(LINUXDIR),linux-2.6.x)
CFLAGS += -DIPLINK_IOCTL_COMPAT=1
endif

RTMONOBJ=rtmon.o

ALLOBJ=$(IPOBJ) $(RTMONOBJ)
SCRIPTS=ifcfg rtpr routel routef
TARGETS=ip rtmon

all: $(TARGETS) $(SCRIPTS) $(LIBS)

ip: $(IPOBJ) $(LIBNETLINK)
	$(CC) $(LDFLAGS) -o $@ $(IPOBJ) $(LIBNETLINK) $(LDLIBS$(LDLIBS_$@))

iplink.o: link.c

link.c: $(KERNEL_INCLUDE)/../.config Makefile
	@printf "$(foreach i,$(LINK_OPT),extern struct link_util $(i)_link_util;\n)" > link.c
	@printf "static struct link_util *dllink[] = {\n" >> link.c
	@printf "$(foreach i,$(LINK_OPT),\t&$(i)_link_util,\n)" >> link.c
	@printf "\tNULL\n};\n" >> link.c

rtmon: $(RTMONOBJ) $(LIBNETLINK)
	$(CC) $(LDFLAGS) -o $@ $(RTMONOBJ) $(LIBNETLINK) $(LDLIBS$(LDLIBS_$@))

romfs:
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_IFCFG  /bin/ifcfg
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_IP     /bin/ip
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_ROUTEF /bin/routef
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_ROUTEL /bin/routel
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_RTMON  /bin/rtmon
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_RTPR   /bin/rtpr

install: all
	install -m 0755 $(TARGETS) $(DESTDIR)$(SBINDIR)
	install -m 0755 $(SCRIPTS) $(DESTDIR)$(SBINDIR)

clean:
	rm -f *.gdb *.o link.c
	rm -f $(ALLOBJ) $(TARGETS)

ifdef not_defined
LDLIBS += -ldl

LDFLAGS += -Wl,-export-dynamic
endif
