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
#include"iopack.h"
#include"plan_sss.h"

namespace Plan{
	freePlanClass::freePlanClass(){
		
	}
	void freePlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void freePlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		outRef.param.kp=getJntKpInit();//outRef为引用，单次更改即可
		outRef.param.kd=getJntKdInit();//outRef为引用，单次更改即可
		outRef.param.maxTor=getJntMaxTorInit();//outRef为引用，单次更改即可
		printEi("j", inSens.j);
		printEi("finger", inSens.fingerJ[0]);
		printEi(inSens.fingerJ[1]);
	}
	bool freePlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		outRef.ctrl.j =inRef.sens.j;
		outRef.ctrl.w.setZero();
		outRef.ctrl.t.setZero();
		outRef.ctrl.enFlag=0;
		outCtrl =outRef.ctrl;
		if(inRef.cmd.key==1){//en按键
			outRef.ctrl.enFlag=1;
			print(planName,"使能，退出");
			return 1;
		}
		return quitFlag;
	}
	void freePlanClass::log(){
		if(logFlag && inSens.cnt%logCnt==0){
			logSS.clear();logSS.str("");
			logSS<<planName<<"\t"<<tim<<"\t";
			logSS<<"tgtJ\t";
			For(NMotMain){logSS<<outCtrl.j[i]<<"\t";}
			logSS<<"actJ\t";
			For(NMotMain){logSS<<inSens.j[i]<<"\t";}

			logSS<<"rpy\t";
			For3{logSS<<inSens.rpy[i]<<"\t";}
			logSS<<"w\t";
			For3{logSS<<inSens.gyr[i]<<"\t";}
			logSS<<"acc\t";
			For3{logSS<<inSens.acc[i]<<"\t";}

			logSS<<"tgtW\t";
			For(NMotMain){logSS<<outCtrl.w[i]<<"\t";}
			logSS<<"actW\t";
			For(NMotMain){logSS<<inSens.w[i]<<"\t";}

			logSS<<"tgtT\t";
			For(NMotMain){logSS<<outCtrl.t[i]<<"\t";}
			logSS<<"actT\t";
			For(NMotMain){logSS<<inSens.t[i]<<"\t";}

			logSS<<"tgtFinger\t";
			For(outCtrl.fingerJ[0].size()){logSS<<outCtrl.fingerJ[0][i]<<"\t";}
			For(outCtrl.fingerJ[1].size()){logSS<<outCtrl.fingerJ[1][i]<<"\t";}
			logSS<<"actFinger\t";
			For(inSens.fingerJ[0].size()){logSS<<inSens.fingerJ[0][i]<<"\t";}
			For(inSens.fingerJ[1].size()){logSS<<inSens.fingerJ[0][i]<<"\t";}

			logger.log(logSS.str());
		}

		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
	}
// ==========================================
	idlePlanClass::idlePlanClass(){
		Ini::iniClass iniArm("../config/arm_base.ini");
		ki=iniArm["jntKiAdd"];
	}
	void idlePlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void idlePlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		outRef.param.kp=getJntKpInit();//outRef为引用，单次更改即可
		outRef.param.kd=getJntKdInit();//outRef为引用，单次更改即可
		outRef.param.maxTor=getJntMaxTorInit();//outRef为引用，单次更改即可
		cout<<"idle:\nin.j:\n";
		For(NMotMain){
			Alg::clip(outCtrl.j[i], inSens.j[i]-0.1, inSens.j[i]+0.1);
			cout<<inSens.j[i]<<", ";
		}
		cout<<"\nout.j:\n";
		For(NMotMain){
			cout<<outCtrl.j[i]<<", ";
		}
		cout<<endl;
		outCtrl.w.setZero();
		errSum.setZero();
	}
	bool idlePlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		dwdate(outRef);
		return quitFlag;
	}
	void idlePlanClass::dwdate(Nabo::outputStruct &outRef){
#ifndef DefSim //实机滤波，仿真不滤波
		For(NMotMain){
			inSens.w[i]=wFil[i].filt(inSens.w[i]);
		}
#endif
		errSum+=outCtrl.j-inSens.j;
		errSum*=0.99;
		if(inSens.cnt<500){
			float rate=inSens.cnt/500.0;
			outCtrl.t=rate*ki*errSum +(1-rate)*outCtrl.t;
		}else{
			outCtrl.t=ki*errSum;
		}
		For(NMotMain){
			outCtrl.t[i]=tOutFil[i].filt(outCtrl.t[i]);
		}
		outRef.ctrl=outCtrl;
	}
	void idlePlanClass::log(){
		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
	}
// ==================================================
	dampPlanClass::dampPlanClass(){
		
	}
	void dampPlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void dampPlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		outRef.param.kp=getJntKpInit();//outRef为引用，单次更改即可
		outRef.param.kd=getJntKdInit();//outRef为引用，单次更改即可
		outRef.param.maxTor=getJntMaxTorInit();//outRef为引用，单次更改即可
		For(NMotMain){
			Alg::clip(outCtrl.j[i], inSens.j[i]-0.1, inSens.j[i]+0.1);
		}
		outCtrl.w.setZero();
	}
	bool dampPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		// out逐渐靠向当前in值
		outCtrl.j=outCtrl.j*0.98 +inSens.j*0.02;
		outCtrl.t*=0.99;
		if(tim>5){
			outCtrl.enFlag=0;
			if(Mng::fsmClass::instance().enforceSwitchWithoutHighCmd("free", "自动")){
				print("damp 5s 自动下使能！");
			}
		}
		outRef.ctrl=outCtrl;
		return quitFlag;
	}
	void dampPlanClass::log(){
		if(logFlag && inSens.cnt%logCnt==0){
			logSS.clear();logSS.str("");
			logSS<<planName<<"\t"<<tim<<"\t";
			logSS<<"tgtJ\t";
			For(NMotMain){logSS<<outCtrl.j[i]<<"\t";}
			logSS<<"actJ\t";
			For(NMotMain){logSS<<inSens.j[i]<<"\t";}

			logSS<<"rpy\t";
			For3{logSS<<inSens.rpy[i]<<"\t";}
			logSS<<"w\t";
			For3{logSS<<inSens.gyr[i]<<"\t";}
			logSS<<"acc\t";
			For3{logSS<<inSens.acc[i]<<"\t";}

			logSS<<"tgtW\t";
			For(NMotMain){logSS<<outCtrl.w[i]<<"\t";}
			logSS<<"actW\t";
			For(NMotMain){logSS<<inSens.w[i]<<"\t";}

			logSS<<"tgtT\t";
			For(NMotMain){logSS<<outCtrl.t[i]<<"\t";}
			logSS<<"actT\t";
			For(NMotMain){logSS<<inSens.t[i]<<"\t";}

			logSS<<"tgtFinger\t";
			For(outCtrl.fingerJ[0].size()){logSS<<outCtrl.fingerJ[0][i]<<"\t";}
			For(outCtrl.fingerJ[1].size()){logSS<<outCtrl.fingerJ[1][i]<<"\t";}
			logSS<<"actFinger\t";
			For(inSens.fingerJ[0].size()){logSS<<inSens.fingerJ[0][i]<<"\t";}
			For(inSens.fingerJ[1].size()){logSS<<inSens.fingerJ[0][i]<<"\t";}

			logger.log(logSS.str());
		}

		if(consoleFlag && inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
	}
// ================================================
}//namespace

