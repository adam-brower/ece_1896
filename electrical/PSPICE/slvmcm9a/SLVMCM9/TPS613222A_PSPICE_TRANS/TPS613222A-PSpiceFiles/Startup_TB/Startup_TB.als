.ALIASES
V_VIN           VIN(+=VIN -=0 ) CN @TPS613222A.Startup_TB(sch_1):INS16760774@SOURCE.VPULSE.Normal(chips)
C_C1            C1(1=VIN 2=0 ) CN @TPS613222A.Startup_TB(sch_1):INS16760820@ANALOG.C.Normal(chips)
X_L1            L1(IN=VIN OUT=SW ) CN @TPS613222A.Startup_TB(sch_1):INS16760716@I2DPARTS.LDCR.Normal(chips)
X_U1            U1(SW=SW VOUT=VOUT GND=0 ) CN
+@TPS613222A.Startup_TB(sch_1):INS16762354@TPS61322DBZ_TRANS.TPS61322DBZR.Normal(chips)
C_C2            C2(1=VOUT 2=0 ) CN @TPS613222A.Startup_TB(sch_1):INS16760999@ANALOG.C.Normal(chips)
R_RLOAD          RLOAD(1=0 2=VOUT ) CN @TPS613222A.Startup_TB(sch_1):INS16761017@ANALOG.R.Normal(chips)
_    _(SW=SW)
_    _(VIN=VIN)
_    _(VOUT=VOUT)
.ENDALIASES
