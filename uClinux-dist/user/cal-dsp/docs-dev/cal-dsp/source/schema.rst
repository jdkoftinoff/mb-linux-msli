
.. |hex| replace:: \ :sub:`16`


.. |dec| replace:: \ :sub:`10`

Address Components
==================

``section``
-----------

Bits 29-28 of the address specify the ``section`` that the address is in. The following values are defined and reserved:

.. tabularcolumns:: |p{5cm}|p{3cm}|p{8cm}|
.. csv-table:: ``section`` codes
   :header: "Token", "Identifier", "Description"
   :widths: 20, 12, 40

   "``meta``", "``0``\ |hex|", "Meta Messages"
   "``enum``", "``1``\ |hex|", "Enumeration of properties and state"
   "``connection``", "``2``\ |hex|", "Media and stream connection management"
   "``control``", "``3``\ |hex|", "Media control"


``subsection``
--------------

Bits 27-26 of the address specify the ``subsection`` that the address is in. The following values are defined and reserved:

.. tabularcolumns:: |p{5cm}|p{3cm}|p{8cm}|
.. csv-table:: ``subsection`` codes
   :header: "Token", "Identifier", "Description"
   :widths: 20, 12, 40

   "``sys``", "``0``\ |hex|", "System oriented values"
   "``talker``", "``1``\ |hex|", "AVTP talker values"
   "``listener``", "``2``\ |hex|", "AVTP listener values"
   "``controller``", "``3``\ |hex|", "AVTP controller values"


``subsubsection``
-----------------

Bits 25-23 of the address specify the ``subsubsection`` that the address is in. The following values are defined and reserved:

.. tabularcolumns:: |p{5cm}|p{3cm}|p{8cm}|
.. csv-table:: ``subsubsection`` codes
   :header: "Token", "Identifier", "Description"
   :widths: 20, 12, 40

   "``device``", "``0``\ |hex|", "Device values"
   "``media``", "``1``\ |hex|", "Media values"
   "``mediafmt``", "``2``\ |hex|", "Media format values"
   "``stream``", "``3``\ |hex|", "Stream values"
   "``streamfmt``", "``4``\ |hex|", "Stream format values"
   "``block``", "``5``\ |hex|", "Processing block values"
   "``vendor``", "``6``\ |hex|", "Vendor specific addresses"
   "``ext``", "``7``\ |hex|", "Subsubsection for future expansion"


``metaaddress``
---------------

When ``section`` is ``meta``, then the bits 15-0 of the address specify the ``metaaddress``. The following values are defined and reserved:

.. tabularcolumns:: |p{5cm}|p{3cm}|p{8cm}|
.. csv-table:: ``metaaddress`` codes
   :header: "Token", "Identifier", "Description"
   :widths: 20, 12, 40

   "``schema/list``", "``20``\ |hex|", "Reserved"
   "``schema/describe``", "``21``\ |hex|", "Reserved"
   "``schema/rw``", "``22``\ |hex|", "Reserved"
   "``time/actuate``", "``30``\ |hex|", "Message modifier to specify gPTP time to actuate the message"
   "``time/record``", "``31``\ |hex|", "Reserved"
   "``time/expire``", "``32``\ |hex|", "Reserved"
   "``time/repeat``", "``33``\ |hex|", "Reserved"
   "``time/duration``", "``34``\ |hex|", "Reserved"
   "``clock/set``", "``40``\ |hex|", "Reserved"
   "``clock/increment``", "``41``\ |hex|", "Reserved"
   "``clock/decrement``", "``42``\ |hex|", "Reserved"
   "``clock/rate``", "``43``\ |hex|", "Reserved"
   "``clock/time``", "``44``\ |hex|", "Reserved"
   "``auth/cred``", "``50``\ |hex|", "Reserved"
   "``auth/salt``", "``51``\ |hex|", "Reserved"
   "``test/ping``", "``60``\ |hex|", "Reserved"
   "``test/pong``", "``61``\ |hex|", "Reserved"
   "``test/checksum``", "``62``\ |hex|", "Reserved"
   "``request/id``", "``70``\ |hex|", "Message modifer to request an acknowledgement of the packet with an identifier"
   "``request/status``", "``71``\ |hex|", "Message modifier that contains the acknowledgement of a packet"
   "``request/return``", "``72``\ |hex|", "Reserved"
   "``request/reflect``", "``73``\ |hex|", "Reserved"
   "``io/report``", "``90``\ |hex|", "Meta message to request values based on a address and address mask"
   "``io/recall``", "``91``\ |hex|", "Reserved"
   "``io/subscribe``", "``92``\ |hex|", "Reserved"
   "``io/refresh``", "``93``\ |hex|", "Reserved"
   "``io/vendor``", "``94``\ |hex|", "Meta message to target specific OUI24 or OUI36 vendor specific addresses"
   "``device/version``", "``400``\ |hex|", "AVDECC Protocol Version"
   "``device/guid``", "``401``\ |hex|", "Device's GUID"
   "``device/vendor``", "``402``\ |hex|", "Vendor's Name"
   "``device/vendoroui``", "``403``\ |hex|", "Vendor's OUI"
   "``device/bootid``", "``404``\ |hex|", "Boot ID"
   "``device/name``", "``405``\ |hex|", "User settable device name"
   "``device/description``", "``406``\ |hex|", "Vendor's description of this device"
   "``device/modelid``", "``407``\ |hex|", "Vendor's model ID"
   "``device/modelname``", "``408``\ |hex|", "Vendor's model name"
   "``device/typeenum``", "``409``\ |hex|", "Device type enumeration"
   "``device/typetxt``", "``40A``\ |hex|", "Textual description of device type"
   "``device/wink``", "``40B``\ |hex|", "Device wink"
   "``device/signal``", "``40C``\ |hex|", "Device signalled"
   "``device/mac``", "``40D``\ |hex|", "Device mac-addr list"
   "``device/gmid``", "``40E``\ |hex|", "Current gPTP gmid list"
   "``device/talkercap``", "``40F``\ |hex|", "Talker capabilities bitmap"
   "``device/listenercap``", "``410``\ |hex|", "Listener capabilities bitmap"
   "``device/controllercap``", "``411``\ |hex|", "Controller capabilities bitmap"


``subaddress``
--------------

When ``section`` is not ``meta``, then the bits 21-16 of the address specify the ``subaddress``. The following values are defined and reserved:

.. tabularcolumns:: |p{5cm}|p{5cm}|
.. csv-table:: ``subaddress`` codes
   :header: "Token", "Identifier"
   :widths: 20, 12

   "``id``", "``0``\ |hex|"
   "``count``", "``1``\ |hex|"
   "``active``", "``2``\ |hex|"
   "``health``", "``3``\ |hex|"
   "``type``", "``4``\ |hex|"
   "``properties``", "``5``\ |hex|"
   "``format``", "``6``\ |hex|"
   "``map``", "``7``\ |hex|"
   "``name``", "``8``\ |hex|"
   "``description``", "``9``\ |hex|"
   "``level``", "``A``\ |hex|"
   "``panpot``", "``B``\ |hex|"
   "``position``", "``C``\ |hex|"
   "``mute``", "``D``\ |hex|"
   "``invert``", "``E``\ |hex|"
   "``pad``", "``F``\ |hex|"
   "``scale``", "``10``\ |hex|"
   "``phantom``", "``11``\ |hex|"
   "``preamp``", "``12``\ |hex|"
   "``trim``", "``13``\ |hex|"
   "``wait``", "``14``\ |hex|"
   "``fade``", "``15``\ |hex|"
   "``pfl``", "``16``\ |hex|"
   "``afl``", "``17``\ |hex|"
   "``meter/map``", "``1A``\ |hex|"
   "``meter/format``", "``19``\ |hex|"
   "``meter/values``", "``18``\ |hex|"
   "``eq``", "``1B``\ |hex|"
   "``dynamics``", "``1C``\ |hex|"
   "``send``", "``1D``\ |hex|"
   "``busassign``", "``1E``\ |hex|"
   "``buslevel``", "``1F``\ |hex|"
   "``effect``", "``20``\ |hex|"
   "``matrix/level``", "``21``\ |hex|"
   "``matrix/mute``", "``22``\ |hex|"
   "``matrix/invert``", "``23``\ |hex|"
   "``matrix/eq``", "``24``\ |hex|"
   "``matrix/delay``", "``25``\ |hex|"
   "``playback/select``", "``26``\ |hex|"
   "``playback/mode``", "``27``\ |hex|"
   "``record/select``", "``28``\ |hex|"
   "``record/mode``", "``29``\ |hex|"
   "``timecode/mode``", "``2A``\ |hex|"
   "``timecode/source``", "``2B``\ |hex|"
   "``timecode/format``", "``2C``\ |hex|"
   "``asrc``", "``2D``\ |hex|"


``item``
--------

Non-``meta`` addresses in form 4 wth bit 22 set contain an 16 bit value in the 16 least significant bits of the address quadlet.

Address Schema
==============



Meta Messages
-------------

Some addresses that are prefixed with "/meta/" modify the handling of the messages following them in the same packet.

.. tabularcolumns:: |p{10cm}|p{5cm}|
.. csv-table:: Meta Messages
   :header: "Address", "Code"

   "``/meta/schema/rw`` ", "``80000022``\ |hex|"
   "``/meta/time/actuate`` ", "``80000030``\ |hex|"
   "``/meta/request/id`` ", "``80000070``\ |hex|"
   "``/meta/request/status`` ", "``80000071``\ |hex|"
   "``/meta/io/report`` ", "``80000090``\ |hex|"
   "``/meta/io/vendor`` ", "``80000094``\ |hex|"



``/meta/schema/rw``
~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/schema/rw

   "Address", "``/meta/schema/rw``"
   "Code", "``80000022``\ |hex|"
   "Parameters", "bool"




Message modifier and response to query if an address is r/w or r/o.

``/meta/time/actuate``
~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/time/actuate

   "Address", "``/meta/time/actuate``"
   "Code", "``80000030``\ |hex|"
   "Parameters", "gptp-time"




Time to actuate messages.

``/meta/request/id``
~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/request/id

   "Address", "``/meta/request/id``"
   "Code", "``80000070``\ |hex|"
   "Parameters", "int32"




Request identifier for request and related response.

``/meta/request/status``
~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/request/status

   "Address", "``/meta/request/status``"
   "Code", "``80000071``\ |hex|"
   "Parameters", "int32"




Status response.

``/meta/io/report``
~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/io/report

   "Address", "``/meta/io/report``"
   "Code", "``80000090``\ |hex|"
   "Parameters", "address-mask address-compare"




Report values of address patterns matching mask and compare value.

``/meta/io/vendor``
~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/io/vendor

   "Address", "``/meta/io/vendor``"
   "Code", "``80000094``\ |hex|"
   "Parameters", "oui24 / oui36"




Specify vendor OUI for any further vendor specific messages.



Device Identity
---------------

Addresses that are prefixed with "/meta/device/" are used for device identity properties

.. tabularcolumns:: |p{10cm}|p{5cm}|
.. csv-table:: Device Identity
   :header: "Address", "Code"

   "``/meta/device/version`` ", "``80000400``\ |hex|"
   "``/meta/device/guid`` ", "``80000401``\ |hex|"
   "``/meta/device/vendor`` ", "``80000402``\ |hex|"
   "``/meta/device/vendoroui`` ", "``80000403``\ |hex|"
   "``/meta/device/bootid`` ", "``80000404``\ |hex|"
   "``/meta/device/name`` ", "``80000405``\ |hex|"
   "``/meta/device/description`` ", "``80000406``\ |hex|"
   "``/meta/device/modelid`` ", "``80000407``\ |hex|"
   "``/meta/device/modelname`` ", "``80000408``\ |hex|"
   "``/meta/device/typeenum`` ", "``80000409``\ |hex|"
   "``/meta/device/typetxt`` ", "``8000040A``\ |hex|"
   "``/meta/device/wink`` ", "``8000040B``\ |hex|"
   "``/meta/device/signal`` ", "``8000040C``\ |hex|"
   "``/meta/device/mac`` ", "``8000040D``\ |hex|"
   "``/meta/device/gmid`` ", "``8000040E``\ |hex|"
   "``/meta/device/talkercap`` ", "``8000040F``\ |hex|"
   "``/meta/device/listenercap`` ", "``80000410``\ |hex|"
   "``/meta/device/controllercap`` ", "``80000411``\ |hex|"



``/meta/device/version``
~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/version

   "Address", "``/meta/device/version``"
   "Code", "``80000400``\ |hex|"
   "Parameters", "int32"




AVDECC Protocol Version.

``/meta/device/guid``
~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/guid

   "Address", "``/meta/device/guid``"
   "Code", "``80000401``\ |hex|"
   "Parameters", "eui64"




Device's GUID.

``/meta/device/vendor``
~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/vendor

   "Address", "``/meta/device/vendor``"
   "Code", "``80000402``\ |hex|"
   "Parameters", "string"




Vendor's human readable name.

``/meta/device/vendoroui``
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/vendoroui

   "Address", "``/meta/device/vendoroui``"
   "Code", "``80000403``\ |hex|"
   "Parameters", "oui24 / oui36"




Vendor's OUI.

``/meta/device/bootid``
~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/bootid

   "Address", "``/meta/device/bootid``"
   "Code", "``80000404``\ |hex|"
   "Parameters", "int64"




Boot identifier.

``/meta/device/name``
~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/name

   "Address", "``/meta/device/name``"
   "Code", "``80000405``\ |hex|"
   "Parameters", "string"




user settable device name.

``/meta/device/description``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/description

   "Address", "``/meta/device/description``"
   "Code", "``80000406``\ |hex|"
   "Parameters", "string"




Human readable product description.

``/meta/device/modelid``
~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/modelid

   "Address", "``/meta/device/modelid``"
   "Code", "``80000407``\ |hex|"
   "Parameters", "int64"




Model identifier.

``/meta/device/modelname``
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/modelname

   "Address", "``/meta/device/modelname``"
   "Code", "``80000408``\ |hex|"
   "Parameters", "string"




Human readable device model name.

``/meta/device/typeenum``
~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/typeenum

   "Address", "``/meta/device/typeenum``"
   "Code", "``80000409``\ |hex|"
   "Parameters", "int32"




Enumerated device types.

``/meta/device/typetxt``
~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/typetxt

   "Address", "``/meta/device/typetxt``"
   "Code", "``8000040A``\ |hex|"
   "Parameters", "string"




Human readable device type.

``/meta/device/wink``
~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/wink

   "Address", "``/meta/device/wink``"
   "Code", "``8000040B``\ |hex|"
   "Parameters", "int32"




Device notification wink request.

``/meta/device/signal``
~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/signal

   "Address", "``/meta/device/signal``"
   "Code", "``8000040C``\ |hex|"
   "Parameters", "int32"




Device notification signalled.

``/meta/device/mac``
~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/mac

   "Address", "``/meta/device/mac``"
   "Code", "``8000040D``\ |hex|"
   "Parameters", "1\*mac-addr"




mac-addr's on this device for each network port.

``/meta/device/gmid``
~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/gmid

   "Address", "``/meta/device/gmid``"
   "Code", "``8000040E``\ |hex|"
   "Parameters", "1\*mac-addr"




mac-addr of current gPTP grand master ID for each network port.

``/meta/device/talkercap``
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/talkercap

   "Address", "``/meta/device/talkercap``"
   "Code", "``8000040F``\ |hex|"
   "Parameters", "int32"




Talker Capabilities.

``/meta/device/listenercap``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/listenercap

   "Address", "``/meta/device/listenercap``"
   "Code", "``80000410``\ |hex|"
   "Parameters", "int32"




Listener Capabilities.

``/meta/device/controllercap``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /meta/device/controllercap

   "Address", "``/meta/device/controllercap``"
   "Code", "``80000411``\ |hex|"
   "Parameters", "int32"




Controller Capabilities.



Media Source Control
--------------------



.. tabularcolumns:: |p{10cm}|p{5cm}|
.. csv-table:: Media Source Control
   :header: "Address", "Code"

   "``/control/talker/media/properties`` ", "``B4850000``\ |hex|"
   "``/control/talker/media/#/type`` ", "``B4C40000``\ |hex| - ``B4C4FFFF``\ |hex|"
   "``/control/talker/media/#/preamp`` ", "``B4D20000``\ |hex| - ``B4D2FFFF``\ |hex|"
   "``/control/talker/media/#/level`` ", "``B4CA0000``\ |hex| - ``B4CAFFFF``\ |hex|"
   "``/control/talker/media/#/mute`` ", "``B4CD0000``\ |hex| - ``B4CDFFFF``\ |hex|"
   "``/control/talker/media/#/phantom`` ", "``B4D10000``\ |hex| - ``B4D1FFFF``\ |hex|"
   "``/control/talker/media/#/asrc`` ", "``B4ED0000``\ |hex| - ``B4EDFFFF``\ |hex|"



``/control/talker/media/properties``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/talker/media/properties

   "Address", "``/control/talker/media/properties``"
   "Code", "``B4850000``\ |hex|"
   "Parameters", "blob"




all media source properties.

``/control/talker/media/#/type``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/talker/media/#/type

   "Address", "``/control/talker/media/#/type``"
   "Code", "``B4C40000``\ |hex| - ``B4C4FFFF``\ |hex|"
   "Parameters", "int32"




media source type.

``/control/talker/media/#/preamp``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/talker/media/#/preamp

   "Address", "``/control/talker/media/#/preamp``"
   "Code", "``B4D20000``\ |hex| - ``B4D2FFFF``\ |hex|"
   "Parameters", "\*bool"




media source preamp enable.

``/control/talker/media/#/level``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/talker/media/#/level

   "Address", "``/control/talker/media/#/level``"
   "Code", "``B4CA0000``\ |hex| - ``B4CAFFFF``\ |hex|"
   "Parameters", "\*gaindb"




media source preamp gain in db - from +10db to +65db.

``/control/talker/media/#/mute``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/talker/media/#/mute

   "Address", "``/control/talker/media/#/mute``"
   "Code", "``B4CD0000``\ |hex| - ``B4CDFFFF``\ |hex|"
   "Parameters", "\*bool"




media source mute.

``/control/talker/media/#/phantom``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/talker/media/#/phantom

   "Address", "``/control/talker/media/#/phantom``"
   "Code", "``B4D10000``\ |hex| - ``B4D1FFFF``\ |hex|"
   "Parameters", "\*bool"




media source phantom power.

``/control/talker/media/#/asrc``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/talker/media/#/asrc

   "Address", "``/control/talker/media/#/asrc``"
   "Code", "``B4ED0000``\ |hex| - ``B4EDFFFF``\ |hex|"
   "Parameters", "\*bool"




media source asynchronous sample rate converter enable.



Media Sink Control
------------------



.. tabularcolumns:: |p{10cm}|p{5cm}|
.. csv-table:: Media Sink Control
   :header: "Address", "Code"

   "``/control/listener/media/properties`` ", "``B8850000``\ |hex|"
   "``/control/listener/media/#/type`` ", "``B8C40000``\ |hex| - ``B8C4FFFF``\ |hex|"
   "``/control/listener/media/#/level`` ", "``B8CA0000``\ |hex| - ``B8CAFFFF``\ |hex|"
   "``/control/listener/media/#/mute`` ", "``B8CD0000``\ |hex| - ``B8CDFFFF``\ |hex|"
   "``/control/listener/media/#/asrc`` ", "``B8ED0000``\ |hex| - ``B8EDFFFF``\ |hex|"



``/control/listener/media/properties``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/listener/media/properties

   "Address", "``/control/listener/media/properties``"
   "Code", "``B8850000``\ |hex|"
   "Parameters", "blob"




all media sink properties.

``/control/listener/media/#/type``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/listener/media/#/type

   "Address", "``/control/listener/media/#/type``"
   "Code", "``B8C40000``\ |hex| - ``B8C4FFFF``\ |hex|"
   "Parameters", "int32"




media sink type.

``/control/listener/media/#/level``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/listener/media/#/level

   "Address", "``/control/listener/media/#/level``"
   "Code", "``B8CA0000``\ |hex| - ``B8CAFFFF``\ |hex|"
   "Parameters", "\*gaindb"




media sink level in db.

``/control/listener/media/#/mute``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/listener/media/#/mute

   "Address", "``/control/listener/media/#/mute``"
   "Code", "``B8CD0000``\ |hex| - ``B8CDFFFF``\ |hex|"
   "Parameters", "\*bool"




media sink mute.

``/control/listener/media/#/asrc``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/listener/media/#/asrc

   "Address", "``/control/listener/media/#/asrc``"
   "Code", "``B8ED0000``\ |hex| - ``B8EDFFFF``\ |hex|"
   "Parameters", "\*bool"




media sink asynchronous sample rate converter enable.



D-Mitri IO Module System Control
--------------------------------

Prefixed by "``/meta/io/vendor`` ``0x001cab``

.. tabularcolumns:: |p{10cm}|p{5cm}|
.. csv-table:: D-Mitri IO Module System Control
   :header: "Address", "Code"

   "``/control/sys/vendor/0x01`` ", "``B3010000``\ |hex|"
   "``/control/sys/vendor/0x02`` ", "``B3020000``\ |hex|"



``/control/sys/vendor/0x01``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/sys/vendor/0x01

   "Address", "``/control/sys/vendor/0x01``"
   "Code", "``B3010000``\ |hex|"
   "Parameters", "int32"




DGPIO relay control.

``/control/sys/vendor/0x02``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. tabularcolumns:: |p{5cm}|p{10cm}|
.. csv-table:: /control/sys/vendor/0x02

   "Address", "``/control/sys/vendor/0x02``"
   "Code", "``B3020000``\ |hex|"
   "Parameters", "int32"




DGPIO DLI status.



