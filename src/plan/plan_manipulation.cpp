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
#include"plan_manipulation.h"

namespace Plan{
	maniPlanClass::maniPlanClass(){
		startCnt=0;
		logFlag=1;
	}
	void maniPlanClass::init(float dt){
		basePlanClass::init(dt);
		startCnt=0;
		filtLevel=0;
		For(NMotMain){
			jFil[i].init(dt,1,0,0,0.2);
		}
	}
	void maniPlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		outRef.param.kp=getJntKpInit();//outRef为引用，单次更改即可
		outRef.param.kd=getJntKdInit();//outRef为引用，单次更改即可
		outRef.param.maxTor=getJntMaxTorInit();//outRef为引用，单次更改即可
		For(NMotMain){
			j0[i]=jFil[i].setBase(inSens.j[i]);
		}
		finger0<<inSens.fingerJ[0], inSens.fingerJ[1];
		// For(NFingerMot){
		// 	j0[i+NArmMot]=jFil[i+NArmMot].setBase(in.j[i+NArmMot]);
		// }
		startCnt=0;
		ctrlMode.reset();
	}
	void maniPlanClass::bye(const Nabo::inputStruct &inRef){
		quitFlag=1;
	}
	bool maniPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);//包含运动学更新与info传出
		update();
		planArm();
		planNeck();
		planLumbar();
		planFinger();
		dwdate(outRef);
		return quitFlag;
	}
	void maniPlanClass::update(){
		switch(inCmd.key){
		case 151://暂停
			startCnt=0;
			cout<<"暂停"<<endl;
			break;
		case 152://开始
			startCnt++;
			cout<<"开始"<<endl;
			break;
		}
		if(startCnt<2)[[unlikely]]{//暂停后直接切模式
			ctrlMode.arm   =(ArmMode)inManiSdk.armMode;
			ctrlMode.neck  =(NeckMode)inManiSdk.neckMode;
			ctrlMode.lumbar=(LumbarMode)inManiSdk.lumbarMode;
			ctrlMode.finger=(FingerMode)inManiSdk.fingerMode;
		}else{
			if(ctrlMode.arm !=(ArmMode)inManiSdk.armMode)[[unlikely]]{
				ctrlMode.arm=(ArmMode)inManiSdk.armMode;
				ctrlMode.armFilRate=1;
				printL("arm mode改变为：",ctrlMode.arm);
			}
			if(ctrlMode.neck !=(NeckMode)inManiSdk.neckMode)[[unlikely]]{
				ctrlMode.neck=(NeckMode)inManiSdk.neckMode;
				ctrlMode.neckFilRate=1;
				printL("neck mode改变为：",ctrlMode.neck);
			}
			if(ctrlMode.lumbar !=(LumbarMode)inManiSdk.lumbarMode)[[unlikely]]{
				ctrlMode.lumbar=(LumbarMode)inManiSdk.lumbarMode;
				ctrlMode.lumbarFilRate=1;
				printL("lumbar mode改变为：",ctrlMode.lumbar);
			}
			if(ctrlMode.finger !=(FingerMode)inManiSdk.fingerMode)[[unlikely]]{
				ctrlMode.finger=(FingerMode)inManiSdk.fingerMode;
				ctrlMode.fingerFilRate=1;
				printL("finger mode改变为：",ctrlMode.finger);
			}
		}
		if(startCnt){
			startCnt++;
		}

		if(inManiSdk.filtLevel !=filtLevel){
			filtLevel=inManiSdk.filtLevel;
			Alg::clip(filtLevel, 0, 5);
			float cutF[6]{1,5,25,40,625,3125};
			float maxV[6]{0.2, 0.4, 0.8, 0.8, 3.2, 6.4};
			printL("滤波级别",filtLevel);
			For(NMotMain){
				// jFil[i].setCutF(cutF[filtLevel], 0, 1.6);
				jFil[i].setCutF(cutF[filtLevel]);
				jFil[i].setMaxDerivate(maxV[filtLevel]);
				// jFil[i].setMaxAcc(0.05);
			}
		}
		// 控制mani是否介入
		outCtrl.inCharge=inManiSdk.inCharge;
	}
	void maniPlanClass::planArm(){
		auto& jStd=getJntStd();
		switch(ctrlMode.arm){
		case ArmRc:
			For(NArmDof){
				Alg::cmd2out1step(jStd[IdArm[0]+i], outCtrl.j[IdArm[0]+i], 0.001);
				Alg::cmd2out1step(jStd[IdArm[1]+i], outCtrl.j[IdArm[1]+i], 0.001);
			}
			break;
		case ArmRoute:
			For(NArmDof){
				outCtrl.j[IdArm[0]+i]=inLocoCmd.j[IdArm[0]+i];
				outCtrl.j[IdArm[1]+i]=inLocoCmd.j[IdArm[1]+i];
			}
			break;
		case ArmJoint:
			For(NArmDof){
				outCtrl.j[IdArm[0]+i]=inManiSdk.armCmd[0][i];
				outCtrl.j[IdArm[1]+i]=inManiSdk.armCmd[1][i];
			}
			break;
		case ArmCart2B:
			For3{
				Alg::clip(inManiSdk.armCmd[0][i], arm.getTipP(0)[i]-0.2, arm.getTipP(0)[i]+0.2);
				Alg::clip(inManiSdk.armCmd[1][i], arm.getTipP(1)[i]-0.2, arm.getTipP(1)[i]+0.2);
			}
			arm.setTip(inManiSdk.armCmd, inManiSdk.armFM);
			arm.dwdate(outCtrl.j.data()+IdArm[0], outCtrl.t.data()+IdArm[0]);
			break;
		case ArmNone://没有break;
		default:
			break;
		}
	}
	void maniPlanClass::planNeck(){
		auto& jStd=getJntStd();
		vec3f dir;
		float neckFilAdd=0.1;//在滤波之前的滤波，由于采用了同一个变量作为中间量，不能简单理解为滤波叠加！
		switch(ctrlMode.neck){
		case NeckRc:
			Alg::cmd2out1step(jStd[IdNeck], outCtrl.j[IdNeck], 0.001);
			Alg::cmd2out1step(jStd[IdNeck+1], outCtrl.j[IdNeck+1], 0.001);
			break;
		case NeckRoute:
			outCtrl.j[IdNeck]=inLocoCmd.j[IdNeck];
			outCtrl.j[IdNeck+1]=inLocoCmd.j[IdNeck+1];
			break;
		case NeckJoint:
			outCtrl.j[IdNeck]=inManiSdk.neckCmd[0];
			outCtrl.j[IdNeck+1]=inManiSdk.neckCmd[1];
			break;
		case NeckNavi:
			outCtrl.j[IdNeck] =outCtrl.j[IdNeck]*neckFilAdd +inCmd.wz*(1-neckFilAdd);
			outCtrl.j[IdNeck+1]*=0.999;
			break;
		case NeckLeftHand:
			dir=arm.getTipP(0) -eyePos;
			outCtrl.j[IdNeck]  =outCtrl.j[IdNeck]*neckFilAdd   +atan2(dir[1],dir[0])*(1-neckFilAdd);
			outCtrl.j[IdNeck+1]=outCtrl.j[IdNeck+1]*neckFilAdd +asin(-dir[2]/dir.norm())*(1-neckFilAdd);
			break;
		case NeckRightHand:
			dir=arm.getTipP(1) -eyePos;
			outCtrl.j[IdNeck]  =outCtrl.j[IdNeck]*neckFilAdd   +atan2(dir[1],dir[0])*(1-neckFilAdd);
			outCtrl.j[IdNeck+1]=outCtrl.j[IdNeck+1]*neckFilAdd +asin(-dir[2]/dir.norm())*(1-neckFilAdd);
			break;
		case NeckMidHand:
			dir=(arm.getTipP(1)+arm.getTipP(2))/2 -eyePos;
			outCtrl.j[IdNeck]  =outCtrl.j[IdNeck]*neckFilAdd   +atan2(dir[1],dir[0])*(1-neckFilAdd);
			outCtrl.j[IdNeck+1]=outCtrl.j[IdNeck+1]*neckFilAdd +asin(-dir[2]/dir.norm() -0.1)*(1-neckFilAdd);
			break;
		case NeckNone://没有break;
		default:
			break;
		}
	}
	void maniPlanClass::planLumbar(){
		auto& jStd=getJntStd();
		switch(ctrlMode.lumbar){
		case LumbarRc:
			For(NLumbarDof){
				Alg::cmd2out1step(jStd[IdLumbar+i], outCtrl.j[IdLumbar+i], 0.001);
			}
			break;
		case NeckRoute:
			For(NLumbarDof){
				outCtrl.j[IdLumbar+i]=inLocoCmd.j[IdLumbar+i];
			}
			break;
		case LumbarJoint:
			For(NLumbarDof){
				outCtrl.j[IdLumbar+i]=inManiSdk.lumbarCmd[i];
			}
			break;
		case LumbarRot:
			//！！！！！暂时还是轴控！！！！！！！
			//！！！！！暂时还是轴控！！！！！！！
			//！！！！！暂时还是轴控！！！！！！！
			For(NLumbarDof){
				outCtrl.j[IdLumbar+i]=inManiSdk.lumbarCmd[i];
			}
			break;
		case LumbarNone://没有break;
		default:
			break;
		}
	}
	void maniPlanClass::planFinger(){
		auto& fingerStd=getFingerStd();

		switch(ctrlMode.finger){
		case FingerRc:
			outCtrl.fingerJ[0]<<fingerStd.head<6>();
			outCtrl.fingerJ[1]<<fingerStd.tail<6>();
			break;
		case FingerRoute:
			outCtrl.fingerJ[0]=inLocoCmd.fingerJ[0];
			outCtrl.fingerJ[1]=inLocoCmd.fingerJ[1];
			break;
		case FingerJoint:
			outCtrl.fingerJ[0]=inManiSdk.fingerCmd[0];
			outCtrl.fingerJ[1]=inManiSdk.fingerCmd[1];
			break;
		case FingerStraight:
			outCtrl.fingerJ[0].setZero();
			outCtrl.fingerJ[1].setZero();
			break;
		case FingerNone://没有break;
		default:
			break;
		}
	}
	void maniPlanClass::dwdate(Nabo::outputStruct &outRef){
		if(startCnt){
			For(NMotMain){
				outCtrl.j[i]=jFil[i].filt(outCtrl.j[i]);
			}
			if(startCnt<1000){
				float rate=startCnt*dt;
				outCtrl.j=outCtrl.j*rate +outRef.ctrl.j*(1-rate);
				outCtrl.fingerJ[0]=outCtrl.fingerJ[0]*rate +outRef.ctrl.fingerJ[0]*(1-rate);
				outCtrl.fingerJ[1]=outCtrl.fingerJ[1]*rate +outRef.ctrl.fingerJ[1]*(1-rate);
			}
			//中途切模式滤波
			if(ctrlMode.armFilRate>0){
				outCtrl.j.segment<NMotArm>(IdArm[0])=(1-ctrlMode.armFilRate)*outCtrl.j.segment<NMotArm>(IdArm[0])
													+ctrlMode.armFilRate*outRef.ctrl.j.segment<NMotArm>(IdArm[0]);
				ctrlMode.armFilRate-=dt;
			}
			if(ctrlMode.neckFilRate>0){
				outCtrl.j.segment<NMotNeck>(IdNeck)=(1-ctrlMode.neckFilRate)*outCtrl.j.segment<NMotNeck>(IdNeck)
													+ctrlMode.neckFilRate*outRef.ctrl.j.segment<NMotNeck>(IdNeck);
				ctrlMode.neckFilRate-=dt;
			}
			if(ctrlMode.lumbarFilRate>0){
				outCtrl.j.segment<NMotLumbar>(IdLumbar)=(1-ctrlMode.lumbarFilRate)*outCtrl.j.segment<NMotLumbar>(IdLumbar)
														+ctrlMode.lumbarFilRate*outRef.ctrl.j.segment<NMotLumbar>(IdLumbar);
				ctrlMode.lumbarFilRate-=dt;
			}
			outRef.ctrl=outCtrl;
		}else{
			outRef.ctrl.w*=0.99;
			outRef.ctrl.t*=0.99;
			outRef.ctrl.fingerW[0]*=0.99;
			outRef.ctrl.fingerW[1]*=0.99;
			outRef.ctrl.fingerT[0]*=0.99;
			outRef.ctrl.fingerT[1]*=0.99;
		}
	}
	void maniPlanClass::log(){
		if(logFlag && inSens.cnt%logCnt==0){
			logSS.clear();logSS.str("");
			logSS<<planName<<"\t"<<tim<<"\t";

			logSS<<"tgtP\t";
			For(inManiSdk.armCmd[0].size()){logSS<<inManiSdk.armCmd[0][i]<<"\t";}
			For(inManiSdk.armCmd[1].size()){logSS<<inManiSdk.armCmd[1][i]<<"\t";}
			logSS<<"tgtJ\t";
			For(outCtrl.j.size()){logSS<<outCtrl.j[i]<<"\t";}
			logSS<<"actJ\t";
			For(inSens.j.size()){logSS<<inSens.j[i]<<"\t";}

			logSS<<"tgtFinger\t";
			For(outCtrl.fingerJ[0].size()){logSS<<outCtrl.fingerJ[0][i]<<"\t";}
			For(outCtrl.fingerJ[1].size()){logSS<<outCtrl.fingerJ[1][i]<<"\t";}
			logSS<<"actFinger\t";
			For(inSens.fingerJ[0].size()){logSS<<inSens.fingerJ[0][i]<<"\t";}
			For(inSens.fingerJ[1].size()){logSS<<inSens.fingerJ[1][i]<<"\t";}

			logger.log(logSS.str());
		}
		if(inSens.cnt%consoleCnt==0){
			cout<<"> "<<planName<<" tim="<<tim<<endl;
		}
		if(inCmd.key == 198){
			actn = Start;
		}
		if(inCmd.key == 199){
			actn = Stop;
		}
		if(actn == Start){
			std::ofstream outFile("上肢各关节角.txt", std::ios::app);
			For(NMotMain){
				outFile << outCtrl.j[i]<<"\t";
			}
			For(6){
				outFile << outCtrl.fingerJ[0][i] <<"\t";
			}
			For(6){
				outFile << outCtrl.fingerJ[1][i] <<"\t";
			}
			outFile << "\n";
			outFile.flush();
			outFile.close();
		}
	}
}//namespace

