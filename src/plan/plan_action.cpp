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
#include"plan_action.h"

namespace Plan{
	actPlanClass::actPlanClass(){
		Ini::iniClass iniAction;
		iniAction.open("../config/arm_action.ini");
		iniAction.getArray("jGreet",  jGreet.data(), NMotMain);
		iniAction.getArray("jRaiseL", jRaiseL.data(),NMotMain);
		iniAction.getArray("jRaiseR", jRaiseR.data(),NMotMain);
		iniAction.getArray("jWaveL",  jWaveL.data(), NMotMain);
		iniAction.getArray("jWaveR",  jWaveR.data(), NMotMain);

		iniAction.getArray("fingerGreet",  fingerGreet.data(), NMotFingerBoth);
		iniAction.getArray("fingerRaiseL", fingerRaiseL.data(),NMotFingerBoth);
		iniAction.getArray("fingerRaiseR", fingerRaiseR.data(),NMotFingerBoth);
		iniAction.getArray("fingerWaveL",  fingerWaveL.data(), NMotFingerBoth);
		iniAction.getArray("fingerWaveR",  fingerWaveR.data(), NMotFingerBoth);

		const std::string sectionName = "recordWholeTrajName";
		recordWholeTrajName = iniAction.getStr(sectionName);
		Ini::splitString(recordWholeTrajName, recordTrajName, ',');

		const std::string sectionPath = "recordTrajKey";
		recordWholeTrajKey = iniAction.getStr(sectionPath);
		Ini::splitString(recordWholeTrajKey, recordTrajKey, ',');
		
		if(recordTrajName.size() != recordTrajKey.size()){
			printL("The number of trajectories is unmatched with the keys !");
		}

		vector<int> keys;
		keys.resize(recordTrajName.size());
		if(recordTrajName.size() > 31){
			printL("The number of trajectory is over limit !");
		}
		iniAction.getArray("recordTrajKey", keys.data(), recordTrajName.size());
		extractNameB4Dot(recordTrajName);
		
		recordTrajNum = recordTrajName.size();
		for (int i = 0; i < recordTrajNum; i++){
			rltvTrajName.emplace_back("../data/"+recordTrajName[i]);
			load(rltvTrajName[i], innerArmActn, innerFingerActn);
			armActnArr.push_back(innerArmActn);
			fingerActnArr.push_back(innerFingerActn);
			printL(i, ": key=", keys[i], ", trajName=", trajName[i]);
			actKeyIds.insert(pair<int,int>(keys[i],i));
			innerArmActn.clear();
			innerFingerActn.clear();
		}
		fingerFist={52,56,90,90,90,90, 52,56,90,90,90,90};
		fistFlag=0;
		logFlag=iniAction["logFlag"];
	}
	void actPlanClass::init(float dt){
		basePlanClass::init(dt);
		For(NMotMain){
			j0[i]=jFil[i].init(dt,2,0,0,0.8);
			jFil[i].setMaxAcc(0.01);
			// j0[i]=jFil[i].init(dt,2,0,0,0.8,2);
		}
	}
	void actPlanClass::hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseReset(inRef,outRef);
		outRef.param.kp=getJntKpInit();//outRef为引用，单次更改即可
		outRef.param.kd=getJntKdInit();//outRef为引用，单次更改即可
		outRef.param.maxTor=getJntMaxTorInit();//outRef为引用，单次更改即可
		cntRcd=0;
		For(NMotMain){
			// j0[i]=jFil[i].reset(2,0,inSens.j[i],1);
			j0[i]=jFil[i].reset(2,0,inSens.j[i]);
			cout<<inRef.sens.j[i]<<", ";
		}
		cout<<endl;
		
		cmdJ=j0.data();
		cmdFinger=finger0.data();
		wave=No;
	}
	void actPlanClass::bye(const Nabo::inputStruct &inRef){
		j0=inRef.sens.j;
		cmdJ=j0.data();
		quitFlag=1;
	}
	bool actPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		baseRun(inRef,outRef);
		update();
		plan();
		dwdate(outRef);
		return quitFlag;
	}
	void actPlanClass::update(){
		if(inCmd.key>=150){
			wave=No;
			actn=Stop;
			fistFlag=0;
			armSwingTime = 0;
			if(inCmd.key == 150){}
			else if(inCmd.key == 151){
				For(NMotMain){
					j0[i]=inSens.j[i];
				}
				finger0<<inSens.fingerJ[0], inSens.fingerJ[1];
				cmdJ=j0.data();
				cmdFinger=finger0.data();
				cout<<"act=暂停\n";
			}else if(inCmd.key == 153){
				j0=getJntStd();
				cmdJ=j0.data();
				finger0=getFingerStd();
				cmdFinger=finger0.data();
				cout<<"act=回正\n";
			}else if(inCmd.key == 154){
				cmdJ=jGreet.data();
				cmdFinger=fingerGreet.data();
				cout<<"act=抱拳\n";
			}else if(inCmd.key == 155){
				cmdJ=jRaiseL.data();
				cmdFinger=fingerRaiseL.data();
				cout<<"act=左抬手\n";
			}else if(inCmd.key == 156){
				cmdJ=jRaiseR.data();
				cmdFinger=fingerRaiseR.data();
				cout<<"act=右抬手\n";
			}else if(inCmd.key == 157){
				wave=Left;
				// cmdJ=j0.data();
				// cmdFinger=finger0.data();
				cntRcd=inSens.cnt;
				cout<<"act=左挥手\n";
			}else if(inCmd.key == 158){
				wave=Right;
				// cmdJ=j0.data();
				// cmdFinger=finger0.data();
				cntRcd=inSens.cnt;
				cout<<"act=右挥手\n";
			}else if(inCmd.key == 159){
				fistFlag=1;
				cout<<"act=握拳\n";
			}else{
				auto it=actKeyIds.find(inCmd.key);
				if(it!=actKeyIds.end()){
					actn = Start;
					actnChce = it->second;
					for (size_t i = 0; i < NMotMain; ++i) {
						cmdJ[i] = armActnArr[actnChce][0][i];
					}
					for (size_t i = 0; i < NMotFingerBoth; ++i) {
						cmdFinger[i] = fingerActnArr[actnChce][0][i];
					}
					armSwingSize = armActnArr[actnChce].size();
					cntRcd = inSens.cnt;
					printL("动作切换为：", trajName[actnChce]);
				}
			}
		}

		if(wave!=No && inSens.cnt-cntRcd>20000){	// 左右挥手结束后，会跳入这个if语句
			wave=No;
			// memcpy(j0,jStd,MemArmHandMot);
			// cmdJ=jRaiseR.data();
			// cmdFinger=fingerRaiseR.data();
			j0=getJntStd();
			cmdJ=j0.data();
			finger0=getFingerStd();
			cmdFinger=finger0.data();
		}
	}
	void actPlanClass::plan(){
		int rem=(inSens.cnt-cntRcd)%2000;
		if(wave==Left){
			if(rem==0){
				cmdJ=jWaveL.data();
				cmdFinger=fingerWaveL.data();
			}else if(rem==1000){
				cmdJ=jRaiseL.data();
				cmdFinger=fingerRaiseL.data();
			}
		}else if(wave==Right){
			if(rem==0){
				cmdJ=jWaveR.data();
				cmdFinger=fingerWaveR.data();
			}else if(rem==1000){
				cmdJ=jRaiseR.data();
				cmdFinger=fingerRaiseR.data();
			}
		}

		int delay = (inSens.cnt-cntRcd)%10;
		if(actn==Start){
			if((armSwingTime <= (armSwingSize-1))){
				// std::cout << "cmdJ1: ";
				if(delay==0){
					// std::cout << "cntRcd1: " << inSens.cnt << std::endl;
					for (size_t i = 0; i < NMotMain; ++i) {
						cmdJ[i] = armActnArr[actnChce][armSwingTime][i];
						// std::cout  << cmdJ[i] << " ";
					}
					for (size_t i = 0; i < NMotFingerBoth; ++i) {
						cmdFinger[i] = fingerActnArr[actnChce][armSwingTime][i];
						// std::cout << "cmdFinger: " << cmdFinger[i] << " " << cmdFinger[i] << std::endl;

					}
					// std::cout << std::endl;
					//cmdFinger=fingerWaveL.data();
					// armSwingTime++;
					armSwingTime = armSwingTime+1;
					// std::cout << "armSwingTime: " << armSwingTime << std::endl;
					// std::cout << "cmdJ: " << cmdJ[0] << " " << cmdJ[1] << " " << cmdJ[2] << " " << cmdJ[3] << " " << cmdJ[4] << " " << cmdJ[5] << " " << cmdJ[6] << std::endl;
					// std::cout << "cmdJ: " << cmdJ[7] << " " << cmdJ[8] << " " << cmdJ[9] << " " << cmdJ[10] << " " << cmdJ[11] << " " << cmdJ[12] << " " << cmdJ[13] << std::endl;
					// std::cout << "cmdJ: " << cmdJ[14] << " " << cmdJ[15] << " " << cmdJ[16] << " " << cmdJ[17] << " " << cmdJ[18] << std::endl;
				}
			}
			// else{
			// 	armSwingTime = 0;
			// }
		}
		// std::cout << "cmdJ2: ";

		For(NMotMain){
			outCtrl.j[i]=jFil[i].filt(cmdJ[i]);
		}

		float*finalFinger=cmdFinger;
		if(fistFlag){
			finalFinger=fingerFist.data();
		}
		if(outCtrl.fingerJ[0].size()==6){
			For(6){
				outCtrl.fingerJ[0][i]=outCtrl.fingerJ[0][i]*0.998 +finalFinger[i]*0.002;
			}
		}else{
			outCtrl.fingerJ[0][0]=outCtrl.fingerJ[0][0]*0.998 +finalFinger[0]*0.002;
		}
		if(outCtrl.fingerJ[1].size()==6){
			For(6){
				outCtrl.fingerJ[1][i]=outCtrl.fingerJ[1][i]*0.998 +finalFinger[i+6]*0.002;
			}
		}else{
			outCtrl.fingerJ[1][0]=outCtrl.fingerJ[1][0]*0.998 +finalFinger[6]*0.002;
		}
	}
	void actPlanClass::dwdate(Nabo::outputStruct &outRef){
		outRef.ctrl=outCtrl;
	}
	void actPlanClass::log(){
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
	void actPlanClass::load(const string& file, vector<array<float,NMotMain>> &armData, vector<array<float,NMotFingerBoth>> &fingerData){
		stringstream ss;
		string line;
		fstream f;
		f.open(file);
		int l=0;
		float raw;
		array<float,NMotMain> tmp;
		array<float,NMotFingerBoth> fingerTmp;
		while(getline(f,line)){
			ss.clear();
			ss.str("");
			ss<<line;
			For(NMotUpper){
				ss>>raw;
				//tmp[i]=raw*0.001/180*Pi;
				if(i<NMotMain){
					tmp[i]=raw;
				}else{
					fingerTmp[i-NMotMain]=raw;
				}
			}
			armData.emplace_back(tmp);
			fingerData.emplace_back(fingerTmp);
			l++;
		}
	}

	void actPlanClass::extractNameB4Dot(vector<string> recordTrajTxtName){
		int trajNum = recordTrajTxtName.size();
		For(trajNum){
			std::string filename = recordTrajTxtName[i];
			size_t dot_pos = filename.find_last_of('.');
		
			// 验证位置有效性
			if (dot_pos != std::string::npos && dot_pos != 0) {
				std::string prefix = filename.substr(0, dot_pos);
				trajName.emplace_back(prefix);
			} else {
				std::cout << "未找到有效分隔符" << std::endl;
			}
		}
	}
}//namespace
