# Makefile to generate various formats from HTML source
#
# Assumes the htmldoc utility is available.
# This can be downloaded from www.easysw.com
#
# Also needs lynx(1) for HTML-to-text conversion

.SUFFIXES: .png .fig

OPENSWANSRCDIR?=$(shell cd .. && pwd)
include ${OPENSWANSRCDIR}/Makefile.inc

# Format arguments for htmldoc
F="--toclevels 4 --header 1cd"

# source files in subdirectory
# basic stuff
a=src/intro.html src/upgrading.html src/quickstart.html \
  src/policygroups.html src/faq.html

# related
b=src/manpages.html src/firewall.html src/trouble.html

# more advanced
c=src/compat.html src/interop.html src/performance.html \
	src/testing.html src/kernel.html src/adv_config.html \
	src/install.html src/config.html \
	src/background.html src/user_examples.html 

# background and reference material
d=src/politics.html src/ipsec.html \
	src/mail.html src/web.html src/glossary.html src/biblio.html \
	src/rfc.html src/roadmap.html

# build and release related
e=src/umltesting.html src/makecheck.html src/nightly.html

sections=$a $b $c $d $e

# separate HTML files built in current directory
separate=intro.html install.html config.html manpages.html \
	firewall.html trouble.html kernel.html roadmap.html \
	compat.html interop.html politics.html ipsec.html \
	mail.html performance.html testing.html web.html \
	glossary.html biblio.html rfc.html faq.html \
	adv_config.html user_examples.html background.html \
	quickstart.html  makecheck.html nightly.html \
        upgrading.html policygroups.html

alldocs=${seperate} index.html 

# where are scripts
SCRIPTDIR=${OPENSWANSRCDIR}/doc/utils

# where
TESTINGDIR=${OPENSWANSRCDIR}/testing

# where do we put HTML manpages?
HMANDIR=manpage.d
SAMPLEMAN=${HMANDIR}/ipsec.8.html

programs: ${alldocs} ${SAMPLEMAN}

all: #$(howto) 

# default, build HTML only
# dependencies build most of it
# then we add index
index.html: ${SAMPLEMAN} ${OPENSWANSRCDIR}/doc/src/index.html
	cp ${OPENSWANSRCDIR}/doc/src/index.html index.html

${SAMPLEMAN}: $(SCRIPTDIR)/mkhtmlman
	-@$(SCRIPTDIR)/mkhtmlman $(HMANDIR) `find ${OPENSWANSRCDIR}/programs ${OPENSWANSRCDIR}/lib ${OPENSWANSRCDIR}/linux -type f -name '*.[1-8]' -print | grep -v lwres | grep -v CVS`

real-all: $(howto) 
remake: $(alldocs) 

realclean:
	@rm -f $(howto) $(separate) toc.html index.html
	@rm -rf $(HMANDIR)

clean:
	@echo nothing here anymore

install: ${alldocs} ${SAMPLEMAN}
	@mkdir -p ${DOCDIR}
	@$(foreach f, $(alldocs), \
		$(INSTALL) $f ${DOCDIR} || exit 1;\
	)
	@find ${HMANDIR} -type f -name "*.html" -print | while read file; \
	do \
		$(INSTALL) $$file ${DOCDIR} || exit 1;\
	done;
	/bin/bash -c 'MANDIR=${MANTREE}; /usr/bin/find ${OPENSWANSRCDIR}/programs ${OPENSWANSRCDIR}/lib ${OPENSWANSRCDIR}/linux -type f -name "*.[1-8]" -print | grep -v lwres | grep -v CVS | while read f; do if [ -f $$f ]; then  manNo=$${f##*.}; F=$${f##*/}; /bin/mkdir -p  $$MANDIR/man$$manNo; /bin/cp -fp $$f $$MANDIR/man$$manNo/ipsec_$$F; fi; done'

install_file_list:
	@$(foreach f, $(alldocs), \
		echo ${DOCDIR}/$f; \
	)
	@if [ -d ${HMANDIR} ]; then find ${HMANDIR} -type f -name "*.html" -print | while read file; \
	do \
		echo ${DOCDIR}/$$file; \
	done; fi;

checkprograms: ;

check: ;

# not enabled by default, because xml2rfc must be installed first.
drafts: draft-richardson-ipsec-opportunistic.txt src/draft-richardson-ipsec-opportunistic.html 

draft-richardson-ipsec-opportunistic.txt: src/draft-richardson-ipsec-opportunistic.xml
	XML_LIBRARY=$(XML_LIBRARY):./src xml2rfc xml2rfc $? $@

draft-%.nr: src/draft-%.xml
	XML_LIBRARY=$(XML_LIBRARY):./src xml2rfc xml2nroff $? $@

draft-%.html: draft-%.xml
	XML_LIBRARY=$(XML_LIBRARY):./src xml2rfc xml2html $? $@


.fig.eps: 
	fig2dev -L ps $< $@

.fig.png: 
	fig2dev -L png $< $@

single_netjig.png: testing/single_netjig.fig
multi_netjig.png: testing/multi_netjig.fig

makecheck.html: single_netjig.png multi_netjig.png

#
# DocBook based documentation
#
xmldocs: mast.html klips/mast.4

mast.html: klips/mast.xml
	xmlto html klips/mast.xml

klips/mast.4: klips/mast.xml
	xmlto -o klips man klips/mast.xml

