#!/bin/metash

proc get_connection_type {} {
	set ethnet [config ref ethernet<devname=eth0>]
	return [config typeof [config get $ethnet conn]]
}

proc set_connection_type {type static_ip static_subnet} {
	set error [catch {
		set ethnet [config ref ethernet<devname=eth0>]
		set conn [config get $ethnet conn]

		if {$type == "dhcp"} {
			config delete $conn
			set newconn [config new dhcp_client]
			config set $newconn name 'LAN'
			config set $newconn interface $ethnet
			config set $ethnet conn $newconn
			config save
		}
		if {$type == "static"} {
			config delete $conn
			set newconn [config new static_ip]
			config set $newconn name 'LAN'
			config set $newconn interface $ethnet
			config set $newconn ipaddr $static_ip
			config set $newconn netmask $static_subnet
			config set $ethnet conn $newconn
			config save
		}
	} output]
	
	if {$error != 0} {
		puts "Error changing network setting to $type:"
		puts "$output"
	}
	
	sleep 3
}

proc display_prompt {} {
	# Display the current network config

	set conn [get_connection_type]
	
	if {[config typeof $conn] == "dhcp_client"} {
		set conntype "DHCP"
	} elseif {[config typeof $conn] == "static_ip"} {
		set conntype "static IP"
	} else {
		set conntype "Unknown! ([config typeof $conn])"
	}
	
	set conndetails	""
	foreach line [split [exec ifconfig | grep -E -A 1 {^eth0[\t ]}] "\n"] {
		if {[regexp {inet addr:([0-9.]+).*Bcast:([0-9.]+).*Mask:([0-9.]+).*} $line dummy ip broadcast mask]} {
			set conndetails "IP: $ip Mask: $mask"
			break
		}
	}
	if {$conndetails == ""} {
		set conndetails "Currently no IP information configured on LAN port"
	}
		
	puts "    Network Config: $conntype - $conndetails"
	puts -nonewline "UTM Emulator > "
}

#############################################################
# Main loop
#

signal handle SIGINT

while 1 {
	display_prompt
	
	# Get the input line	
	set error [catch {
		gets stdin input
	} result]
	if {$error == 1} {
		puts "An error occurred reading the input: $result"
		continue
	}

	if {[string equal "" $input]} {
		continue
	}

	set input [string tolower $input]
	set conntype [get_connection_type]
	
	if {$input == "dhcp"} {
		if {$conntype == "dhcp_client"} {
			puts "Already in DHCP mode"
		} else {
			puts "Changing network to DHCP mode"
			set_connection_type dhcp "" ""
		}
	} elseif {[regexp {([0-9.]+)/([0-9]+)} $input dummy ip subnet]} {
		puts "Changing network to static IP/subnet of $ip/$subnet"
		set_connection_type static $ip $subnet
	} elseif {$input == "exit"} {
		puts "Exiting"
		exit
	} elseif {[string length "$input"] > 0} {
		puts "Unknown command: '$input'"
		puts "    Either enter dhcp to change network to DHCP assigned IP,"
		puts "    or enter an IP address/subnet mask in the form aaa.bbb.ccc.ddd/ee"
		puts "    e.g. 192.168.0.1/24"
		puts ""
	}
	flush stdout
}
