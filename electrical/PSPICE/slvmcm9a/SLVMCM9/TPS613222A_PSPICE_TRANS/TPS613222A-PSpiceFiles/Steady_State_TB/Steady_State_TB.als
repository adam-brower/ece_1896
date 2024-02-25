.ALIASES
C_C2            C2(1=VOUT 2=0 ) CN @TPS613222A.Steady_State_TB(sch_1):INS16761353@ANALOG.C.Normal(chips)
V_V1            V1(+=VIN -=0 ) CN @TPS613222A.Steady_State_TB(sch_1):INS16761562@SOURCE.VDC.Normal(chips)
R_RLOAD          RLOAD(1=0 2=VOUT ) CN @TPS613222A.Steady_State_TB(sch_1):INS16761383@ANALOG.R.Normal(chips)
C_C1            C1(1=VIN 2=0 ) CN @TPS613222A.Steady_State_TB(sch_1):INS16761409@ANALOG.C.Normal(chips)
X_L1            L1(IN=VIN OUT=SW ) CN @TPS613222A.Steady_State_TB(sch_1):INS16761429@I2DPARTS.LDCR.Normal(chips)
X_U1            U1(SW=SW VOUT=VOUT GND=0 ) CN
+@TPS613222A.Steady_State_TB(sch_1):INS16762191@TPS61322DBZ_TRANS.TPS61322DBZR.Normal(chips)
_    _(SW=SW)
_    _(VIN=VIN)
_    _(VOUT=VOUT)
.ENDALIASES
