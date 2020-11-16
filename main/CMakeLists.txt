set(COMPONENT_SRCS 
				# Platform_SRC:
					"main.c"
					"port/bacnet.c"
                	"port/device.c"
					"port/dlmstp.c"
					"port/led.c"
					"port/mstimer-init.c"
					"port/netport.c"
					"port/rs485.c" 

				# BASIC_SRC:
					"bacnet/basic/service/h_dcc.c"
					"bacnet/basic/service/h_apdu.c"
					"bacnet/basic/npdu/h_npdu.c"
					"bacnet/basic/service/h_rd.c"
					"bacnet/basic/service/h_rp.c"
					"bacnet/basic/service/h_rpm.c"
					"bacnet/basic/service/h_whohas.c"
					"bacnet/basic/service/h_whois.c"
					"bacnet/basic/service/h_wp.c"
					"bacnet/basic/service/h_noserv.c"
					"bacnet/basic/service/s_iam.c"
					"bacnet/basic/service/s_ihave.c"
					"bacnet/basic/tsm/tsm.c"
					"bacnet/basic/sys/ringbuf.c"
					"bacnet/basic/sys/fifo.c"
					"bacnet/basic/sys/mstimer.c"

				#BACNET_SRC:
					"bacnet/abort.c"
					"bacnet/bacaddr.c"
					"bacnet/bacapp.c"
					"bacnet/bacdcode.c"
					"bacnet/bacerror.c"
					"bacnet/bacint.c"
					"bacnet/bacreal.c"
					"bacnet/bacstr.c"
					"bacnet/datalink/crc.c"
					"bacnet/dcc.c"
					"bacnet/iam.c"
					"bacnet/ihave.c"
					"bacnet/lighting.c"
					"bacnet/memcopy.c"
					"bacnet/npdu.c"
					"bacnet/proplist.c"
					"bacnet/rd.c"
					"bacnet/reject.c"
					"bacnet/rp.c"
					"bacnet/rpm.c"
					"bacnet/whohas.c"
					"bacnet/whois.c"
					"bacnet/wp.c"
					
				   )

set(COMPONENT_ADD_INCLUDEDIRS "." "port/include")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

register_component()
