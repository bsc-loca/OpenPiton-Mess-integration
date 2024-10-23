#include <stdio.h>
#include <stdlib.h>
#include "bw_lat_mem_ctrl.h"


#ifndef PITON_DPI
#include "veriuser.h"
#include "acc_user.h"
#else
#ifdef VERILATOR
#include "verilated_vpi.h"
#endif
#endif


#ifdef PITON_DPI
#include "svdpi.h"
#endif


using namespace std;

static BwLatMemCtrl* bwLatMemCtrl =NULL;
static unsigned long long int access_cnt = 0;


// Routines called by the verilog code.
#ifndef PITON_DPI
extern "C" void delay_init_call();
extern "C" void got_a_write_req_call();
extern "C" void got_a_read_req_call();
extern "C" void get_Bandwidth_call();
#else // ifndef PITON_DPI
extern "C" void delay_init_call(const char * curve_path, int freq);
extern "C" unsigned long long  got_a_write_req_call(unsigned long long accessCycle);
extern "C" unsigned long long  got_a_read_req_call(unsigned long long accessCycle);
extern "C" double  get_Bandwidth_call(void);
#endif

#ifdef PITON_DPI
double get_Bandwidth_call(void){
	//cout << "bandwdith from function " << bwLatMemCtrl->getBandwidth() << " GB/s" <<endl;
	return bwLatMemCtrl->getBandwidth();
}
#else
void get_Bandwidth_call( ){
	double bw =  bwLatMemCtrl->getBandwidth();
	
	tf_putrealp(0, bw);	
	return;
}
#endif


void access_monitor (unsigned long long  lat,unsigned long long accessCycle){
	access_cnt++;
	if(access_cnt%64==0) {
		//printf("**Info: lat=%llu,accessCycle=%llu,accessNumber=%llu\n",lat,accessCycle,access_cnt);
	}
}


#ifndef PITON_DPI
void delay_init_call(){
	string curve_path =tf_getcstringp(1);  // a get file name.
	int freq = tf_getp(2);
#else // ifndef PITON_DPI
void delay_init_call(const char * curve_path, int freq){ //frequency in MHz
    // Convert const char* to std::string
    std::string curve_path_string(curve_path);	
#endif
	if(bwLatMemCtrl != NULL) return;
	double fr_ghz = (double)freq/1000;
	bwLatMemCtrl = new BwLatMemCtrl(curve_path, 1000, 50, fr_ghz);
	if(bwLatMemCtrl == NULL){
		cout << "**Info: Error loading delay model:" << curve_path << "curve\n";
		return;
	}
	cout << "**Info: Delay model is initilized with:\"" << curve_path << "\" curve & " << fr_ghz << " GHz\n";
}


#ifdef PITON_DPI
unsigned long long  got_a_write_req_call ( unsigned long long accessCycle ) {
	unsigned long long lat = (unsigned long long ) bwLatMemCtrl->access(accessCycle,1);
	access_monitor (lat, accessCycle);
	return lat;
}

unsigned long long  got_a_read_req_call (unsigned long long accessCycle ) {
	unsigned long long lat =  (unsigned long long ) bwLatMemCtrl->access(accessCycle,0);
	access_monitor (lat, accessCycle);
	return lat;
}
#else 

void got_a_write_req_call (  ) {
	unsigned long long accessCycle;
    int low, high;
  	low  = tf_getlongp(&high, 1);
  	accessCycle   = high;
  	accessCycle <<= 32;
  	accessCycle  |= (unsigned)low;
	
	unsigned long long  lat =  bwLatMemCtrl->access(accessCycle,1);
	low = lat & 0xffffffff;
    high = (lat >> 32) & 0xffffffff;
    tf_putlongp(2, low, high);	
	access_monitor (lat, accessCycle);
    return;
}

void got_a_read_req_call (  ) {
	unsigned long long accessCycle;
    int low, high;
  	low  = tf_getlongp(&high, 1);
  	accessCycle   = high;
  	accessCycle <<= 32;
  	accessCycle  |= (unsigned)low;
  		
	unsigned long long  lat = bwLatMemCtrl->access(accessCycle,0);
    low = lat & 0xffffffff;
    high = (lat >> 32) & 0xffffffff;
    tf_putlongp(2, low, high);	
	access_monitor (lat, accessCycle);
    return;
}

#endif




