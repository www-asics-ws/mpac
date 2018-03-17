
all:	sim
SHELL = /bin/sh
MS=-s

##########################################################################
#
# DUT Sources
#
##########################################################################
#DUT_SRC_DIR=../../../rtl/verilog
DUT_SRC_DIR=../rtl

_TARGETS_=							\
								\
		$(DUT_SRC_DIR)/mpac_top.v			\
		$(DUT_SRC_DIR)/mpac_axl_if.v			\
		$(DUT_SRC_DIR)/mpac_buffer.v			\
		$(DUT_SRC_DIR)/mpac_pctrl.v			\
		$(DUT_SRC_DIR)/mpac_ser_io.v			\
		$(DUT_SRC_DIR)/mpac_rf.v			\
		$(DUT_SRC_DIR)/mpac_data_mux.v			\
		$(DUT_SRC_DIR)/mpac_dpram.v			\
		$(DUT_SRC_DIR)/mpac_fifo_dc_ext_mem.v		\
		$(DUT_SRC_DIR)/mpac_sync.v			\


##########################################################################
#
# Test Bench Sources
#
##########################################################################
#TB_SRC_DIR=../../../bench/verilog
TB_SRC_DIR=../bench


_TB_=						\
	$(TB_SRC_DIR)/test_bench_top.v		\
	$(TB_SRC_DIR)/axi_istr_bfm.v		\
	$(TB_SRC_DIR)/axi_ostr_bfm.v		\
	$(TB_SRC_DIR)/axl_mast_bfm.v		\
	$(TB_SRC_DIR)/i2s_bfm.v			\



##########################################################################
#
# Misc Variables
#
##########################################################################

INCDIR= +incdir+./$(DUT_SRC_DIR)/ +incdir+./$(TB_SRC_DIR)/ 
LOGF=-l .nclog

HAL_INCDIR= -incdir $(DUT_SRC_DIR) -incdir $(TB_SRC_DIR)
HAL_NOCHECK=STYVAL,NOBLKN,BITUSD,CNSTLT,IMPTYP,USEPRT,USEWIR,USEREG,BADSYS,NETDCL,LCVARN,CTLCHR,IDLENG,UCCONN,SEPLIN,MAXLEN,DLNBLK,DECLIN,CBPAHI,TPOUNR,FFWASR,MPCMPE,FFASRT,SYNASN,SYNPRT,ASNRST,URDPRT,URDWIR,POIASG,UNCONO

##########################################################################
#
# Make Targets
#
##########################################################################
.PHONY: ss cim simw clean

ss:
	simvision -INPUT waves/simvision.sv &

simw:
	$(MAKE) $(MS) sim ACCESS="+access+r " WAVES="+define+WAVES"

sim:
	@rm -rf trace_*
	@ncverilog -q +define+TEST_BENCH $(_TARGETS_) $(_TB_)		\
		$(INCDIR) $(WAVES) $(ACCESS) $(LOGF) +ncstatus		\
		+nowarn+PARRNG +define+U3D_TESTING +nc64bit 		\
	
	@if  grep -i "warn" .nclog > /dev/null;					\
	then echo "[1;37;43mWARNINGS DETECTED[0m";				\
	fi
	@if  grep -i "error" .nclog | grep -v "Total ERRORS: 0" > /dev/null;	\
	then echo "[1;37;41mERRORS DETECTED[0m";				\
	else echo "[1;37;42mALL TESTS PASSED[0m";				\
	fi

hal:
	@hal -NOCHECK $(HAL_NOCHECK) $(HAL_INCDIR) -TOP u3d_top -RACES -LOGFILE u3d_hal.log $(_TARGETS_)


clean:
	rm -rf INCA_libs ncverilog.key ncverilog.log hal* u3d_hal.log waves/waves.dsn waves/waves.trn .nclog

##########################################################################
#       ESC Sequences
#       =============
#            Invert: \033[7m
#              Bold: \033[1m
#             Blink: \033[5m
#    Red Background: \033[30;41m
# Orange Background: \033[30;43m
#  Green Background: \033[30;42m
#        Cancel all: \033[0m

test_color:
	@echo "[30;41mALL TESTS PASSED[0m";
	@echo "[30;42mALL TESTS PASSED[0m";
	@echo "[30;43mALL TESTS PASSED[0m";
	@echo "[30;44mALL TESTS PASSED[0m";
	@echo "[30;45mALL TESTS PASSED[0m";
	@echo "[30;46mALL TESTS PASSED[0m";
	@echo "[30;47mALL TESTS PASSED[0m";

	@echo "[30;41;1mALL TESTS PASSED[0m";
	@echo "[30;42;1mALL TESTS PASSED[0m";
	@echo "[30;43;1mALL TESTS PASSED[0m";
	@echo "[30;44;1mALL TESTS PASSED[0m";
	@echo "[30;45;1mALL TESTS PASSED[0m";
	@echo "[30;46;1mALL TESTS PASSED[0m";
	@echo "[30;47;1mALL TESTS PASSED[0m";

	@echo "[37;41mALL TESTS PASSED[0m";
	@echo "[37;42mALL TESTS PASSED[0m";
	@echo "[37;43mALL TESTS PASSED[0m";
	@echo "[37;44mALL TESTS PASSED[0m";
	@echo "[37;45mALL TESTS PASSED[0m";
	@echo "[37;46mALL TESTS PASSED[0m";
	@echo "[37;47mALL TESTS PASSED[0m";

	@echo "[1;37;41mALL TESTS PASSED[0m";
	@echo "[1;37;42mALL TESTS PASSED[0m";
	@echo "[1;37;43mALL TESTS PASSED[0m";
	@echo "[1;37;44mALL TESTS PASSED[0m";
	@echo "[1;37;45mALL TESTS PASSED[0m";
	@echo "[1;37;46mALL TESTS PASSED[0m";
	@echo "[1;37;47mALL TESTS PASSED[0m";

	@echo "[1;7;37;41mALL TESTS PASSED[0m";
	@echo "[1;7;37;42mALL TESTS PASSED[0m";
	@echo "[1;7;37;43mALL TESTS PASSED[0m";
	@echo "[1;7;37;44mALL TESTS PASSED[0m";
	@echo "[1;7;37;45mALL TESTS PASSED[0m";
	@echo "[1;7;37;46mALL TESTS PASSED[0m";
	@echo "[1;7;37;47mALL TESTS PASSED[0m";


	@echo "[1;5;37;41mALL TESTS PASSED[0m";
	@echo "[1;5;37;42mALL TESTS PASSED[0m";
	@echo "[1;5;37;43mALL TESTS PASSED[0m";
	@echo "[1;5;37;44mALL TESTS PASSED[0m";
	@echo "[1;5;37;45mALL TESTS PASSED[0m";
	@echo "[1;5;37;46mALL TESTS PASSED[0m";
	@echo "[1;5;37;47mALL TESTS PASSED[0m";



