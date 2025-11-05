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
#include"plan_recover.h"

namespace Plan{
	rcPlanClass::rcPlanClass(){
		Ini::iniClass iniArm("../config/arm_base.ini");
		logFlag=iniArm["rcLogFlag"];
		ki=iniArm["jntKiAdd"];
		tRc=2;
	}
	void rcPlanClass::init(float dt){
		basePlanClass::init(dt);
	}
	void rcPlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		outRef.param.kp=getJntKpInit();//outRef为引用，单次更改即可
		outRef.param.kd=getJntKdInit();//outRef为引用，单次更改即可
		outRef.param.maxTor=getJntMaxTorInit();//outRef为引用，单次更改即可
		outRef.param.fingerMaxT=getFingerMaxTorInit();//outRef为引用，单次更改即可
		errSum.setZero(NMotMain);
		j0=inSens.j;
		finger0<<inSens.fingerJ[0], inSens.fingerJ[1];
	}
	bool rcPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		plan();
		dwdate(outRef);
		return quitFlag;
	}
	void rcPlanClass::plan(){
		auto &jStd=getJntStd();
		auto &fingerStd=getFingerStd();
		if(tim<tRc){
			float pgs=tim/tRc;
			float s=0.5-cos(pgs*Pi)*0.5;
			float sd=Pi*0.5*sin(pgs*Pi)/tRc;
			outCtrl.j=j0+s*(jStd-j0);
			outCtrl.w=sd*(jStd-j0);
			
			if(outCtrl.fingerJ[0].size()==6){
				outCtrl.fingerJ[0]=finger0.head<6>() +s*(fingerStd.head<6>()-finger0.head<6>());
				outCtrl.fingerW[0]=sd*(fingerStd.head<6>()-finger0.head<6>());
			}
			if(outCtrl.fingerJ[1].size()==6){
				outCtrl.fingerJ[1]=finger0.tail<6>() +s*(fingerStd.tail<6>()-finger0.tail<6>());
				outCtrl.fingerW[1]=sd*(fingerStd.tail<6>()-finger0.tail<6>());
			}
		}else{quitFlag=1;}
	}
	void rcPlanClass::dwdate(Nabo::outputStruct &outRef){
		errSum+=outCtrl.j-inSens.j;
		errSum*=0.99;
		outCtrl.t=ki*errSum;//底层还有一套pd，此处为增强闭环效果
		For(NMotMain){
			outCtrl.t[i]=tOutFil[i].filt(outCtrl.t[i]);
		}
		// For(NMotFinger){
		// 	outCtrl.fingerT[i]=0;
		// }
		outRef.ctrl=outCtrl;
	}
	void rcPlanClass::log(){
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
}//namespace

