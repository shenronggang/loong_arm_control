/*
Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============

=====================================================*/
#include"plan.h"

namespace Plan{
	basePlanClass::basePlanClass(){
		consoleFlag=1;
		Ini::iniClass iniArm;
		iniArm.open("../config/arm_base.ini");
		logFlag=iniArm["logFlag"];
		iniArm.getArray("jntStd",jntStd.data(),NMotMain);
		iniArm.getArray("jntKpInit",jntKpInit.data(),NMotMain);
		iniArm.getArray("jntKdInit",jntKdInit.data(),NMotMain);
		iniArm.getArray("jntMaxTorInit",jntMaxTorInit.data(),NMotMain);
		iniArm.getArray("fingerStd",fingerStd.data(),NMotFingerBoth);
		fingerMaxTorInit=iniArm["fingerMaxTorInit"];
		logger.init("mani");
	}
	void basePlanClass::init(float dtt){
		dt=dtt;
		arm.setDt(dt);
		logCnt=int(0.01/dt+0.001);//0.01s一帧log
		consoleCnt=int(1/dt+0.001);//一帧控制台print

		For(NMotMain){
			jFil[i].init(dt,50);
			wFil[i].init(dt,50);
			tInFil[i].init(dt,30);
			tOutFil[i].init(dt,30);
		}
	}
	void basePlanClass::baseReset(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		inCmd=inRef.cmd;
		inManiSdk=inRef.maniSdk;
		inSens=inRef.sens;
		tim=0;
		quitFlag=0;
		For(NMotMain){
			wFil[i].reset(30,0,0);
			jFil[i].reset(30,0,0);
#ifdef DefSim //仿真滤波
			tInFil[i].reset(30,0,inSens.t[i]);
			tOutFil[i].reset(30,0,0);
#else //实机滤波（在底层处理了）
			tInFil[i].reset(30,0,inSens.t[i]);
			tOutFil[i].reset(30,0,0);
#endif
		}
		outRef.param.kp=jntKpInit;//outRef为引用，单次更改即可
		outRef.param.kd=jntKdInit;//outRef为引用，单次更改即可
		outRef.param.maxTor=jntMaxTorInit;//outRef为引用，单次更改即可
		outCtrl=outRef.ctrl;
		cout<<planName<<" plan hey!\n";
	}
	void basePlanClass::baseRun(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		inCmd=inRef.cmd;
		inManiSdk=inRef.maniSdk;
		inSens=inRef.sens;
		tim=inSens.cnt*dt;
		// 遥控权限最高，overide，控制mani是否介入(底层还会再做一次判断)
		if(inRef.cmd.key==220){
			outCtrl.inCharge=0;
		}else if(inRef.cmd.key==221){
			outCtrl.inCharge=1;
		}

		arm.update(inSens.j.data()+IdArm[0], inSens.w.data()+IdArm[0], inSens.t.data()+IdArm[0]);
		For2{
			for(int j=0;j<3;j++){
				outRef.info.actTipPRpy2B[i][j]=arm.getTipP(i)[j];
				outRef.info.actTipPRpy2B[i][j+3]=arm.getTipRpy(i)[j];

				outRef.info.tgtTipPRpy2B[i][j]=arm.getTgtTipP(i)[j];
				outRef.info.tgtTipPRpy2B[i][j+3]=arm.getTgtTipRpy(i)[j];
			}
			for(int j=0;j<6;j++){
				outRef.info.actTipVW2B[i][j]=arm.getTipVW(i)[j];
				outRef.info.actTipFM2B[i][j]=arm.getTipFM(i)[j];
				outRef.info.tgtTipVW2B[i][j]=arm.getTgtTipVW(i)[j];
				outRef.info.tgtTipFM2B[i][j]=arm.getTgtTipFM(i)[j];
			}
		}
	}
}//namespace

