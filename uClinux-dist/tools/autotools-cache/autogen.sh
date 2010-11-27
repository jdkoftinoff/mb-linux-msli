#!/bin/sh -x
exec env AUTOMAKE='automake --foreign' autoreconf -f -i
