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
#include"plan_main_frame.h"

namespace Plan{
	framePlanClass::framePlanClass(){
		Ini::iniClass ini;
		ini.open("../config/arm_frame.ini");
		string tmp=ini.getStr("files");
		vector<string> files;
		Ini::splitString(tmp,files,',');
		load(files);
		logFlag=ini["logFlag"];
		// cutF=ini["cutF"];
		noteFile.open("../log/记录帧.csv", ios::out);
		noteFile<<"[键值]\n[说明]\n帧时间秒 +左臂7 +右臂7 +颈2 +腰3 +左指6 +右指6 +注释\n";
	}
	void framePlanClass::load(vector<string> files){
		stringstream ss;
		string line;
		fstream f;
		for(auto & file:files){
			f.open("../data/"+file);
			if(!f){
				printL(file,"无法打开");
				continue;
			}
			// 此处存在严重风险，需要改进
			actionStruct act;
			f>>act.key;
			f>>act.name;
			printL("frame key=", act.key, " name=", act.name, " file=", file);
			getline(f,line);//跳过注释行
			while(getline(f,line)){
				// print(line.size(), line);
				if(line.size()<50){continue;}
				ss.clear();
				ss.str("");
				ss<<line;
				act.frames.emplace_back(frameStruct());
				auto &frame=act.frames.back();
				ss>>frame.duration;
				Alg::clip(frame.duration, 0.1, 10);
				For(NMotMain){
					ss>>frame.jnt[i];
				}
				For(NMotFingerBoth){
					ss>>frame.finger[i];
				}
			}
			store.insert(make_pair(act.key, act));
			// for(auto fr:act.frames){
			// 	cout<<"frame:";
			// 	For(NMotMain){
			// 		cout<<fr.jnt[i]<<",";
			// 	}
			// 	For(NMotFingerBoth){
			// 		cout<<fr.finger[i]<<",";
			// 	}
			// 	cout<<endl;
			// }
			f.close();
		}
	}

	void framePlanClass::init(float dt){
		basePlanClass::init(dt);
		For(NMotMain){
			// j0[i]=jFil[i].init(dt,cutF,0,0,0.8);
			j0[i]=jFil[i].init(dt,2,0,0,0.8);
			jFil[i].setMaxAcc(0.005);
			// j0[i]=jFil[i].init(dt,2,0,0,0.8,2);
		}
	}
	void framePlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		outRef.param.kp=getJntKpInit();//outRef为引用，单次更改即可
		outRef.param.kd=getJntKdInit();//outRef为引用，单次更改即可
		outRef.param.maxTor=getJntMaxTorInit();//outRef为引用，单次更改即可
		For(NMotMain){
			j0[i]=jFil[i].reset(2,0,inSens.j[i]);
			// j0[i]=jFil[i].reset(cutF,0,inSens.j[i]);
			cout<<inRef.sens.j[i]<<", ";
		}
		jFil[4].setCutF(5);
		jFil[4].setMaxDerivate(2);
		jFil[4].setMaxAcc(0.02);
		jFil[11].setCutF(5);
		jFil[11].setMaxDerivate(2);
		jFil[11].setMaxAcc(0.02);
		//腰
		jFil[18].setCutF(0.3);

		cout<<endl;
		cmdJ=j0.data();
		cmdFinger=finger0.data();
		actionNow=nullptr;
	}
	void framePlanClass::bye(const Nabo::inputStruct &inRef){
		j0=inRef.sens.j;
		cmdJ=j0.data();
		quitFlag=1;
	}
	bool framePlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		update();
		plan();
		dwdate(outRef);
		return quitFlag;
	}
	void framePlanClass::update(){
		if(inCmd.key>=150){
			if(inCmd.key == 150){}
			else if(inCmd.key == 151){
				// For(NMotMain){
				// 	j0[i]=inSens.j[i];
				// }
				// finger0<<inSens.fingerJ[0], inSens.fingerJ[1];
				// cmdJ=j0.data();
				// cmdFinger=finger0.data();
				// cout<<"act=暂停\n";
			}else if(inCmd.key == 153){
				actionNow=nullptr;
				cout<<"act=回正\n";
			}else{
				auto it=store.find(inCmd.key);
				if(it!=store.end()){
					frameIdNow=0;
					actionNow=&(it->second);
					cmdJ=(actionNow->frames[0]).jnt;
					cmdFinger=(actionNow->frames[0]).finger;
					cntRcd=inSens.cnt;
					frameDurationCnt=(actionNow->frames[0]).duration/dt;
					printL("act=", actionNow->name);
				}
			}
			if(inCmd.key==158){
				static int noteCnt=0;
				printL("记录点",noteCnt);
				noteFile<<"1.0\t";
				For(NMotMain){
					noteFile<<inSens.j[i]<<"\t";
				}
				For(NMotFingerLeft){
					noteFile<<inSens.fingerJ[0][i]<<"\t";
				}
				For(NMotFingerRight){
					noteFile<<inSens.fingerJ[1][i]<<"\t";
				}
				noteFile<<":"<<noteCnt<<endl;
				noteCnt++;
			}
		}
		if(actionNow==nullptr){
			j0=getJntStd();
			cmdJ=j0.data();
			finger0=getFingerStd();
			cmdFinger=finger0.data();


			// auto it=store.find(154);
			// if(it!=store.end()){
			// 	frameIdNow=0;
			// 	actionNow=&(it->second);
			// 	cmdJ=(actionNow->frames[0]).jnt;
			// 	cmdFinger=(actionNow->frames[0]).finger;
			// 	cntRcd=inSens.cnt;
			// 	printL("act=", actionNow->name);
			// }
		}
	}
	void framePlanClass::plan(){
		if(actionNow){
			int cnt=inSens.cnt-cntRcd;
			if(cnt==frameDurationCnt){
				frameIdNow++;
				if(frameIdNow==actionNow->frames.size()){
					printL(actionNow->name,"结束");
					actionNow=nullptr;
					frameIdNow=0;
				}else{
					cmdJ=(actionNow->frames[frameIdNow]).jnt;
					cmdFinger=(actionNow->frames[frameIdNow]).finger;
					frameDurationCnt=(actionNow->frames[frameIdNow]).duration/dt;
					cntRcd=inSens.cnt;
				}
			}
		}

		For(NMotMain){
			outCtrl.j[i]=jFil[i].filt(cmdJ[i]);
		}
		For(NMotFingerLeft){
			outCtrl.fingerJ[0][i]=cmdFinger[i];
			outCtrl.fingerJ[1][i]=cmdFinger[NMotFingerLeft+i];
		}
	}
	void framePlanClass::dwdate(Nabo::outputStruct &outRef){
		outRef.ctrl=outCtrl;
	}
	void framePlanClass::log(){
		if(logFlag && inSens.cnt%logCnt==0){
			logSS.clear();logSS.str("");
			logSS<<planName<<"\t"<<tim<<"\t";

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
	}
}//namespace
