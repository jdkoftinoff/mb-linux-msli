#!/bin/sh
# This assumes a linux target

GNUCONFIG=${0%/*}/config.sub
tuple=$1

if [ -z "${tuple}" ] ; then
	echo "Usage: $0 <tuple>" 1>&2
	exit 1
fi

header_path() {
	echo "#include <$1>" | ${cc} -E - | grep -o "\".*/$1\"" | \
		sed \
			-e 's:"::g' \
			-e "s:/$1::" \
			-e 'q'
}

var_filter() { sed -e 's:[-/.+]:_:g' "$@" ; }
ac_var_filter() { var_filter -e "s:^:ac_cv_$1_:" ; }
ac_header_filter() { ac_var_filter header "$@" ; }
ac_func_filter() { ac_var_filter func "$@" ; }

gnu_tuple="$(${GNUCONFIG} ${tuple})"
cc="${tuple}-gcc"
nm="${tuple}-nm"
readelf="${tuple}-readelf"

inc_libc_path=$(header_path stdio.h)
inc_gcc_path=$(header_path stddef.h)
printf "Using inc dir for $tuple (${gnu_tuple}):\n\t%s\n\t%s\n" "$inc_libc_path" "$inc_gcc_path" 1>&2

find $inc_libc_path $inc_gcc_path -name '*.h' -printf '%P=yes\n' | ac_header_filter > ${gnu_tuple}
invalid=$(sed -e 's:[a-zA-Z0-9=_]::g' ${gnu_tuple})
if [ -n "${invalid}" ] ; then
	(
	echo "ERROR: generated invalid cache var"
	echo "${invalid}" | sort -u
	) 1>&2
	exit 1
fi

headers="
	dlfcn.h
	ioctls.h
	locale.h
	stropts.h
	wctype.h
	asm/reg.h
	bits/dlfcn.h
	sys/reg.h
	sys/stream.h
"
for header in ${headers} ; do
	var=$(echo ${header} | ac_header_filter)
	if ! grep -qs ${var}=yes ${gnu_tuple} ; then
		echo "${var}=no" >> ${gnu_tuple}
	fi
done

${cc} -print-file-name=libc.a | \
	xargs ${readelf} -s | \
	awk '($4 == "FUNC" && $5 != "LOCAL" && $6 == "DEFAULT") { print $NF }' | \
	sed -e 's:^:ac_cv_func:' -e 's:$:=yes:' >> ${gnu_tuple}

for func in fork sys_siglist ; do
	var=$(echo ${func} | ac_func_filter)
	if ! grep -qs ${var}=yes ${gnu_tuple} ; then
		echo "${var}=no" >> ${gnu_tuple}
	fi
done

	cat <<-EOF >> ${gnu_tuple}
ac_cv_host=${gnu_tuple}
ac_cv_target=${gnu_tuple}

ac_cv_prog_AR=${tuple}-ar
ac_cv_prog_CC=${tuple}-gcc
ac_cv_prog_CPP='${tuple}-gcc -E'
ac_cv_prog_CXX=${tuple}-g++
ac_cv_prog_CXXCPP='${tuple}-g++ -E'
ac_cv_prog_F77=${tuple}-gfortran
ac_cv_prog_OBJDUMP=${tuple}-objdump
ac_cv_prog_RANLIB=${tuple}-ranlib
ac_cv_prog_STRIP=${tuple}-strip

ac_cv_c_const=yes
ac_cv_c_compiler_gnu=yes
ac_cv_c_inline=inline
am_cv_CC_dependencies_compiler_type=none
ac_cv_cxx_compiler_gnu=yes
am_cv_CXX_dependencies_compiler_type=none
ac_cv_exeext=
ac_cv_f77_compiler_gnu=yes
ac_cv_objext=o
ac_cv_prog_cc_g=yes
ac_cv_prog_cxx_g=yes
ac_cv_prog_f77_g=yes

# Magic detection changes over time and libtool versions.  This cache
# works fine with newer libtool (2.x+), but breaks with older ones.
#lt_cv_deplibs_check_method=unknown
#lt_cv_file_magic_cmd='\$MAGIC_CMD'
#lt_cv_file_magic_test_file=
lt_cv_ld_reload_flag=-r
lt_cv_nm_interface='BSD nm'
lt_cv_objdir=.libs
lt_cv_path_LD=${tuple}-ld
lt_cv_path_LDCXX=${tuple}-ld
lt_cv_path_NM=${tuple}-nm
lt_cv_prog_compiler_c_o=yes
lt_cv_prog_compiler_c_o_CXX=yes
lt_cv_prog_compiler_c_o_F77=yes
lt_cv_prog_compiler_pic_works=yes
lt_cv_prog_compiler_rtti_exceptions=no
lt_cv_prog_compiler_static_works=yes
lt_cv_prog_gnu_ld=yes
lt_cv_prog_gnu_ldcxx=yes
EOF

echo ${gnu_tuple}
