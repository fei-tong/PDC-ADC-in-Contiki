CONTIKI = ../../../../contiki
CONTIKI_PROJECT = example-mesh example-collect example-trickle example-polite hello-world\
				  example-rudolph0 example-rudolph1 example-rudolph2 example-rucb \
			      example-runicast example-unicast example-neighbors hello-world-micaz
APPS+=powertrace
DEFINES=PROJECT_CONF_H=\"project-conf.h\"

PDCMACDIR=..
PROJECTDIRS += $(PDCMACDIR) 
TIMEDATE=`date +%Y%m%d-%H%M`

#for local version pdcadc
CONTIKIDIRS += $(PDCMACDIR)/net/mac/pdcadc
# original: pdcadc_board.c
CONTIKI_SOURCEFILES += pdcadc.c 
				
# to configure CFLAGS
CFLAGS+= -iquote -Werror

#Synchronization settings
#1: will do synchronization based on the proposed strategy; 0: no synchronization
PRI_SYNC_SUPPORT ?= 1  
CFLAGS+= -DCMD_PRI_SYNC_SUPPORT=$(PRI_SYNC_SUPPORT)

#for cooja simulation, if PRI_SYNC_SUPPORT==0, this variable will not work, 1: generate random drift; 0: no drift
RANDOM_DRIFT_IN_COOJA_SUPPORT ?= 1
CFLAGS+= -DCMD_RANDOM_DRIFT_IN_COOJA_SUPPORT=$(RANDOM_DRIFT_IN_COOJA_SUPPORT)

#Address related
#process the grade message according to address for real board (set to 1) or transmission range, for cooja simulation (set to 0)
PROC_GRADE_BASED_ON_ADDR_SUPPORT ?= 1
CFLAGS+= -DCMD_PROC_GRADE_BASED_ON_ADDR_SUPPORT=$(PROC_GRADE_BASED_ON_ADDR_SUPPORT)
#NODES_NUM_PER_GRADE is valid only when PROC_GRADE_BASED_ON_ADDR_SUPPORT = 1
NODES_NUM_PER_GRADE ?= 2 
CFLAGS+= -DCMD_NODES_NUM_PER_GRADE=$(NODES_NUM_PER_GRADE)

ID_FOR_Z_ONE ?= 1 #The ID assigned to the Z1 1.0 mote
CFLAGS+= -DCMD_ID_FOR_Z_ONE=$(ID_FOR_Z_ONE)
ID_FOR_Z_TWO ?= 2 #The ID assigned to the Z1 2.0 mote
CFLAGS+= -DCMD_ID_FOR_Z_TWO=$(ID_FOR_Z_TWO)
ID_FOR_Z_THREE ?= 3 #The ID assigned to the Z1 3.0 mote
CFLAGS+= -DCMD_ID_FOR_Z_THREE=$(ID_FOR_Z_THREE)
ID_FOR_Z_FOUR ?= 4 #The ID assigned to the Z1 4.0 mote
CFLAGS+= -DCMD_ID_FOR_Z_FOUR=$(ID_FOR_Z_FOUR)
ID_FOR_Z_FIVE ?= 5 #The ID assigned to the Z1 5.0 mote
CFLAGS+= -DCMD_ID_FOR_Z_FIVE=$(ID_FOR_Z_FIVE)

ID_FOR_MICAZ_ONE ?= 1 #The ID assigned to the MICAZ mote 1
CFLAGS+= -DCMD_ID_FOR_MICAZ_ONE=$(ID_FOR_MICAZ_ONE)
ID_FOR_MICAZ_TWO ?= 2 #The ID assigned to the MICAZ mote 2
CFLAGS+= -DCMD_ID_FOR_MICAZ_TWO=$(ID_FOR_MICAZ_TWO)
ID_FOR_MICAZ_THREE ?= 4 #The ID assigned to the MICAZ mote 3
CFLAGS+= -DCMD_ID_FOR_MICAZ_THREE=$(ID_FOR_MICAZ_THREE)
ID_FOR_MICAZ_FOUR ?= 6 #The ID assigned to the MICAZ mote 4
CFLAGS+= -DCMD_ID_FOR_MICAZ_FOUR=$(ID_FOR_MICAZ_FOUR)
ID_FOR_MICAZ_FIVE ?= 7 #The ID assigned to the MICAZ mote 5
CFLAGS+= -DCMD_ID_FOR_MICAZ_FIVE=$(ID_FOR_MICAZ_FIVE)

#every PERIODIC_CYCLES cycles, send RTS
PERIODIC_CYCLES ?= 3 #every 3 cycles, send RTS 
CFLAGS+= -DCMD_PERIODIC_CYCLES=$(PERIODIC_CYCLES)
#maximum cycles for keeping active after determining its grade.
MAX_ACTIVE_CYCLES ?= 3
CFLAGS+= -DCMD_MAX_ACTIVE_CYCLES=$(MAX_ACTIVE_CYCLES)
#when the node grade is or becomes -1, the node will wait such number +1 of receivings to determine its new grade
RCV_GRADE_NUM_TO_SET ?= 2
CFLAGS+= -DCMD_RCV_GRADE_NUM_TO_SET=$(RCV_GRADE_NUM_TO_SET)
#START_GRADE, unit: second
START_GRADE ?= 10
CFLAGS+= -DCMD_START_GRADE=$(START_GRADE)
#GEN_DATA_IN_R: 1: generate data in R status; 0: generate data in function pri_qsend_packet;*/
GEN_DATA_IN_R ?=0
CFLAGS+= -DCMD_GEN_DATA_IN_R=$(GEN_DATA_IN_R)

#support multiple platforms
CROSS_PLATFORM_SUPPORT ?= 1
CFLAGS+= -DCMD_CROSS_PLATFORM_SUPPORT=$(CROSS_PLATFORM_SUPPORT)

#Data related
DATA_TX_SUPPORT ?= 1 #1:has data to be sent; 0: no data
CFLAGS+= -DCMD_DATA_TX_SUPPORT=$(DATA_TX_SUPPORT)

##############################################################
RETX_LIMIT ?= 6 #6,allow 5 retransmissions, =0: no limit; >0: transmission times, i.e., (RETX_LIMIT-1) retransmissions
CFLAGS+= -DCMD_RETX_LIMIT=$(RETX_LIMIT)

#queue buffer size
MAX_DATA_QUEUE_BUFS ?= 10 
CFLAGS+= -DCMD_MAX_DATA_QUEUE_BUFS=$(MAX_DATA_QUEUE_BUFS)

#for sim, 15pkt/node, for board, 90pkt/node, if DATA_TX_SUPPORT==1, this varialbe indicates how many data a node can generate
#if TOTAL_DATA_NUM<0, node will generate data endlessly
TOTAL_DATA_NUM ?= 50
CFLAGS+= -DCMD_TOTAL_DATA_NUM=$(TOTAL_DATA_NUM) 

################################################# 
#1: ADC, support adaptive duty cycle, 0: PDC, doesn't support
ADCSC_SUPPORT ?= 1
CFLAGS+= -DCMD_ADCSC_SUPPORT=$(ADCSC_SUPPORT)
 
#sleep factor(18, 22, 26)
SF ?= 10
CFLAGS+= -DCMD_SF=$(SF)

ADDRESS_FREE_SUPPORT ?= 0
CFLAGS+= -DCMD_ADDRESS_FREE_SUPPORT=$(ADDRESS_FREE_SUPPORT)

#unit seconds  
DATA_INTERVAL ?= 9
CFLAGS+= -DCMD_DATA_INTERVAL=$(DATA_INTERVAL)
#################################################

IS_COOJA_SIM ?= 1
CFLAGS+= -DCMD_IS_COOJA_SIM=$(IS_COOJA_SIM)

CFLAGS += -ffunction-sections 
LDFLAGS +=-Wl,--gc-sections,--undefined=_reset_vector__,--undefined=InterruptVectors,--undefined=_copy_data_init__,--undefined=_clear_bss_init__,--undefined=_end_of_init__

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_RIME = 1
CONTIKI_NO_NET = 1 #to exclude all the networking code. 
include $(CONTIKI)/Makefile.include

