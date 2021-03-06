#
# (C) Tenable Network Security
#
# This script contains information extracted from VuXML :
#
# Copyright 2003-2006 Jacques Vidrine and contributors
#
# Redistribution and use in source (VuXML) and 'compiled' forms (SGML,
# HTML, PDF, PostScript, RTF and so forth) with or without modification,
# are permitted provided that the following conditions are met:
# 1. Redistributions of source code (VuXML) must retain the above
#   copyright notice, this list of conditions and the following
#   disclaimer as the first lines of this file unmodified.
# 2. Redistributions in compiled form (transformed to other DTDs,
#   published online in any format, converted to PDF, PostScript,
#   RTF and other formats) must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer
#   in the documentation and/or other materials provided with the
#   distribution.
#
# THIS DOCUMENTATION IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
# OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS DOCUMENTATION,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
#
 seealso  = '\n';

if ( description )
{
 script_id(14757);
 script_version("$Revision: 1.4 $");
 script_cve_id("CVE-2004-0722");

 script_name(english:"FreeBSD : mozilla -- SOAPParameter integer overflow (117)");


desc["english"] = "
The remote host is missing an update to the system

The following package is affected: firefox

Solution : Update the package on the remote host
See also : " + seealso; 
 script_description(english:desc["english"]);
 script_summary(english:"Check for firefox");
 script_category(ACT_GATHER_INFO);
 script_copyright(english:"This script is Copyright (C) 2006 Tenable Network Security");
 family["english"] = "FreeBSD Local Security Checks";
 script_family(english:family["english"]);
 script_dependencies("ssh_get_info.nasl");
 script_require_keys("Host/FreeBSD/pkg_info");
 exit(0);
}

include('freebsd_package.inc');


pkg_test(pkg:"firefox<0.9",
     url:"http://www.FreeBSD.org/ports/portaudit/a4fd8f53-05eb-11d9-b45d-000c41e2cdad.html",
     problem:'mozilla -- SOAPParameter integer overflow',
     seealso:seealso);

pkg_test(pkg:"linux-mozilla<1.7",
     url:"http://www.FreeBSD.org/ports/portaudit/a4fd8f53-05eb-11d9-b45d-000c41e2cdad.html",
     problem:'mozilla -- SOAPParameter integer overflow',
     seealso:seealso);

pkg_test(pkg:"linux-mozilla-devel<1.7",
     url:"http://www.FreeBSD.org/ports/portaudit/a4fd8f53-05eb-11d9-b45d-000c41e2cdad.html",
     problem:'mozilla -- SOAPParameter integer overflow',
     seealso:seealso);

pkg_test(pkg:"mozilla-gtk1<1.7",
     url:"http://www.FreeBSD.org/ports/portaudit/a4fd8f53-05eb-11d9-b45d-000c41e2cdad.html",
     problem:'mozilla -- SOAPParameter integer overflow',
     seealso:seealso);

pkg_test(pkg:"mozilla<1.7,2",
     url:"http://www.FreeBSD.org/ports/portaudit/a4fd8f53-05eb-11d9-b45d-000c41e2cdad.html",
     problem:'mozilla -- SOAPParameter integer overflow',
     seealso:seealso);

pkg_test(pkg:"netscape7<7.2",
     url:"http://www.FreeBSD.org/ports/portaudit/a4fd8f53-05eb-11d9-b45d-000c41e2cdad.html",
     problem:'mozilla -- SOAPParameter integer overflow',
     seealso:seealso);
