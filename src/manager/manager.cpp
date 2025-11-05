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
#include"manager.h"
#include"plan_sss.h"
#include"plan_recover.h"
#include"plan_action.h"
#include"plan_main_frame.h"
#include"plan_manipulation.h"
#include"timing.h"
#include"fsm.h"
#include"iopack.h"

enum KeyEnum:int{
	KeyY=1,
	KeyX=2,
	KeyA=3,
	KeyB=4,
	KeyLL=7,
	KeyRR=8,
	KeyDown=13,
	KeyUp=15,
	KeyRight=16,
	KeyLeft=17,
};

// ================================
namespace Mng{
class manageClass::impClass{
public:
	impClass();
	float jntLow[NMotMain],jntUp[NMotMain];
	// float fingerLow[NMotFingerBoth],fingerUp[NMotFingerBoth];
	Plan::freePlanClass free;
	Plan::idlePlanClass idle;
	Plan::dampPlanClass damp;
	Plan::rcPlanClass rc;
	Plan::actPlanClass act;
	Plan::framePlanClass frame;
	Plan::maniPlanClass mani;
	fsmClass &fsm=fsmClass::instance();

	void step(Nabo::inputStruct &in,Nabo::outputStruct &out);
};
	manageClass::impClass::impClass(){
		// ------------------
		// 首先addInit添加初始plan
		// 其次addIdle添加空闲plan。某plan退出但未指定下一plan时，将自动进入空闲plan
		// 然后addCommon再添加其他plan
		// enforce=1：当需切换本plan时，无需等待正在运行的plan退出，强制转换为本plan；
		// enforce=0：当需切换本plan时，正在运行的plan退出后，转换为本plan；
		// ------------------
#ifdef DefSim //仿真简化，借用fms idle的自动特性，直接切入目标plan
		fsm.addInit(rc,900,"rc",1);
		// fsm.addIdle(act,901,"act",0);
		fsm.addIdle(frame,902,"frame",0);
		// fsm.addIdle(mani,902,"mani",0);
		// fsm.addIdle(idle,KeyA);
#else
		fsm.addInit(free,KeyDown);
		fsm.addIdle(idle,KeyA);
		fsm.addCommon(idle,112,"idle2",1);//同一个plan，不一样的key
		fsm.addCommon(damp,12,"damp1",1);
		fsm.addCommon(damp,113,"damp",1);//同一个plan，不一样的key
		fsm.addCommon(rc,KeyX,"rc1",0);
		fsm.addCommon(rc,114,"rc",0);//同一个plan，不一样的key
		fsm.addCommon(act,115,"act",0);
		fsm.addCommon(mani,116,"mani",0);
		fsm.addCommon(frame,4,"frame",0);
#endif
		Ini::iniClass iniArm;
		iniArm.open("../config/arm_base.ini");
		iniArm.getArray("jntLimitLow",jntLow,NMotMain);
		iniArm.getArray("jntLimitUp",jntUp,NMotMain);
		// iniArm.getArray("fingerLimitLow",fingerLow,NMotFingerBoth);
		// iniArm.getArray("fingerLimitUp",fingerUp,NMotFingerBoth);
	}
	void manageClass::impClass::step(Nabo::inputStruct &in,Nabo::outputStruct &out){
		fsm.step(in,out);
		For(NMotMain){
			Alg::clip(out.ctrl.j[i], jntLow[i], jntUp[i]);
			Alg::clip(out.ctrl.t[i], out.param.maxTor[i]);
		}
		// For(NMotFingerBoth){
		// 	Alg::clip(out.ctrl.fingerJ[i], fingerLow[i], fingerUp[i]);
		// 	Alg::clip(out.ctrl.fingerT[i], out.param.fingerMaxT);
		// }
	}
//=====================================================
	manageClass::manageClass():imp(*new impClass()){}
	manageClass& manageClass::instance(){
		static manageClass singtn;
		return singtn;
	}
	void manageClass::init(float dt){
		imp.fsm.init(dt);
		cout<<"manager: dt="<<dt<<endl;
	}
	void manageClass::step(Nabo::inputStruct &in,Nabo::outputStruct &out){
		imp.step(in,out);
	}
}//namespace











		// if(in.key!=13 && in.key<60 || in.key==keyOld){
		// 	in.key=-1;
		// }else{
		// 	keyOld=in.key;
		// 	cout<<"nabo:key="<<in.key<<endl;
		// }
		// switch(in.key){
		// case KeyDown://dis
		// case 60://dis
		// 	in.cnt=0;
		// 	state=Dis;
		// 	print("Down: ds");
		// 	break;
		// case 61://en
		// 	if(state==Dis){
		// 		in.cnt=0;
		// 		state=Idle;
		// 		print("Y: en");
		// 	}
		// 	break;
		// case 62://rc
		// 	if(state==Idle){
		// 		in.cnt=0;
		// 		state=Rc;
		// 		print("X: rc");
		// 	}
		// 	break;
		// case 63://idle
		// 	// print("B: no");
		// 	in.cnt=0;
		// 	state=Idle;
		// 	break;
		// case 64://act
		// 	if(state==Idle){
		// 		in.cnt=0;
		// 		state=Act;
		// 		print("Right: act");
		// 	}
		// 	break;
		// }

		// double ms;
		// switch(state){
		// case Idle:
		// 	idle.run(in,out);
		// 	break;
		// case Rc:
		// 	if(rc.run(in,out)){//退出为1
		// 		state=Idle;
		// 		in.cnt=-1;
		// 	};
		// 	rc.log();
		// 	break;
		// case Act:
		// 	act.run(in,out);
		// 	act.log();
		// 	break;
		// default:
		// 	memcpy(out.j, in.j, MemArmHandMot);
		// 	break;
		// }
		// // For(NArmMot){
		// // 	Alg::clip(out.j[i], MotLow[i], MotUp[i]);
		// // }
		// in.cnt++;