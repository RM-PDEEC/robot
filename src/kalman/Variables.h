#ifndef VARIABLES_H
#define VARIABLES_H

#include "Kalman.h"

Matrix real_positions = {
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {-0.000028184, 0.000000007},
    {0.000056369, 0.000000026},
    {0.000760958, 0.000004826},
    {0.000958228, 0.000007652},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000986409, 0.000008109},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000930047, 0.000007209},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.000958228, 0.000007652},
    {0.005014280, 0.000146290},
    {0.009512280, 0.000467995},
    {0.015140600, 0.001111920},
    {0.021455400, 0.002139770},
    {0.035477601, 0.005528210},
    {0.042776000, 0.007873900},
    {0.050218999, 0.010661900},
    {0.057607200, 0.013814100},
    {0.072242901, 0.021151699},
    {0.079363301, 0.025221899},
    {0.086375102, 0.029530600},
    {0.093309902, 0.034065999},
    {0.100063004, 0.038719799},
    {0.106719002, 0.043512199},
    {0.113266997, 0.048404299},
    {0.119723998, 0.053369399},
    {0.126105994, 0.058385100},
    {0.132404000, 0.063416399},
    {0.138651997, 0.068464004},
    {0.144866005, 0.073509797},
    {0.157216996, 0.083516099},
    {0.163350001, 0.088437200},
    {0.169489995, 0.093304798},
    {0.175666004, 0.098126002},
    {0.187955007, 0.107436001},
    {0.194190994, 0.111991003},
    {0.200512007, 0.116475001},
    {0.206854001, 0.120832004},
    {0.219825998, 0.129265994},
    {0.226366997, 0.133264005},
    {0.232987002, 0.137130007},
    {0.238701001, 0.140309006},
    {0.243333995, 0.142763004},
    {0.247925997, 0.145048007},
    {0.252770990, 0.147262007},
    {0.263740003, 0.151473999},
    {0.269856006, 0.153353006},
    {0.276302010, 0.154991999},
    {0.283100009, 0.156358004},
    {0.297479004, 0.158079997},
    {0.304942012, 0.158377007},
    {0.312550992, 0.158269003},
    {0.320226997, 0.157744005},
    {0.335660011, 0.155412003},
    {0.343286008, 0.153613999},
    {0.350883991, 0.151377007},
    {0.358374000, 0.148718998},
    {0.372943997, 0.142168999},
    {0.380005985, 0.138340995},
    {0.386916012, 0.134302005},
    {0.393689007, 0.130089998},
    {0.400269002, 0.125681996},
    {0.406691998, 0.120999999},
    {0.412921995, 0.116020001},
    {0.424605012, 0.105205998},
    {0.430029988, 0.099359401},
    {0.435144007, 0.093237899},
    {0.439942986, 0.086832300},
    {0.448449999, 0.073343903},
    {0.452147990, 0.066277102},
    {0.455462992, 0.059022401},
    {0.458384991, 0.051600799},
    {0.463019997, 0.036313001},
    {0.464720994, 0.028491599},
    {0.466001987, 0.020590300},
    {0.466861010, 0.012632200},
    {0.467292994, 0.004639490},
    {0.467357010, -0.003420970},
    {0.467168987, -0.011423100},
    {0.466762990, -0.019388899},
    {0.466087997, -0.027308200},
    {0.465097010, -0.035194200},
    {0.463753015, -0.043056302},
    {0.462040991, -0.050817799},
    {0.457432002, -0.066113703},
    {0.454566002, -0.073496401},
    {0.451281011, -0.080796003},
    {0.447629005, -0.087886900},
    {0.439186990, -0.101481996},
    {0.434450001, -0.107899003},
    {0.429358006, -0.114074998},
    {0.423942000, -0.119969003},
    {0.412317008, -0.131135002},
    {0.406282008, -0.136436000},
    {0.400110006, -0.141488999},
    {0.393761009, -0.146270007},
    {0.387205005, -0.150763005},
    {0.380443007, -0.154938996},
    {0.373457015, -0.158790007},
    {0.360042006, -0.164955005},
    {0.355012000, -0.166872993},
    {0.350243002, -0.168519005},
    {0.345378995, -0.170056000},
    {0.334984988, -0.172968999},
    {0.329353005, -0.174377993},
    {0.323367000, -0.175762996},
    {0.317164987, -0.177085996},
    {0.303923994, -0.179569006},
    {0.297053009, -0.180686995},
    {0.289941996, -0.181731999},
    {0.282676011, -0.182686001},
    {0.267800003, -0.184316993},
    {0.260192007, -0.184995994},
    {0.252548009, -0.185579002},
    {0.244812995, -0.186072007},
    {0.237045005, -0.186477005},
    {0.229245007, -0.186798006},
    {0.221413001, -0.187037006},
    {0.205772996, -0.187277004},
    {0.197908998, -0.187289998},
    {0.190045998, -0.187235996},
    {0.182183996, -0.187115997},
    {0.166435003, -0.186676994},
    {0.158549994, -0.186358005},
    {0.150694996, -0.185975999},
    {0.142845005, -0.185526997},
    {0.127157003, -0.184423000},
    {0.119291998, -0.183770999},
    {0.111461997, -0.183054000},
    {0.103638001, -0.182265997},
    {0.095849402, -0.181410000},
    {0.088041700, -0.180475995},
    {0.080300003, -0.179468006},
    {0.072569497, -0.178377002},
    {0.064824298, -0.177193001},
    {0.057094801, -0.175909996},
    {0.050491799, -0.174728006},
    {0.045421101, -0.173785001},
    {0.036020700, -0.172245994},
    {0.031007599, -0.171680003},
    {0.025608899, -0.171308994},
    {0.019804100, -0.171191007},
    {0.007005700, -0.171926007},
    {0.000129266, -0.172812998},
    {-0.007059290, -0.173907995},
    {-0.014418200, -0.175007999},
    {-0.029460501, -0.176660001},
    {-0.037117202, -0.177040994},
    {-0.044783302, -0.177055001},
    {-0.052496199, -0.176670998},
    {-0.060205702, -0.175872996},
    {-0.067943603, -0.174640998},
    {-0.075573303, -0.172986999},
    {-0.083153099, -0.170893997},
    {-0.090604097, -0.168381006},
    {-0.097904399, -0.165459007},
    {-0.105057999, -0.162126005},
    {-0.112042002, -0.158390999},
    {-0.125487998, -0.149713993},
    {-0.131837994, -0.144841999},
    {-0.137940004, -0.139616996},
    {-0.143730998, -0.134092003},
    {-0.154400006, -0.122126997},
    {-0.159229994, -0.115744002},
    {-0.163774997, -0.109121002},
    {-0.168144003, -0.102413997},
    {-0.176746994, -0.088913597},
    {-0.180848002, -0.082104899},
    {-0.184744000, -0.075145103},
    {-0.188339993, -0.068088897},
    {-0.191634998, -0.060855899},
    {-0.194582999, -0.053474899},
    {-0.197152004, -0.045983501},
    {-0.201122999, -0.030626100},
    {-0.202497005, -0.022797801},
    {-0.203455001, -0.014879500},
    {-0.203988999, -0.006921150},
    {-0.203823999, 0.009110510},
    {-0.203349993, 0.017157299},
    {-0.202831998, 0.025173100},
    {-0.202249005, 0.033128001},
    {-0.200498000, 0.048925102},
    {-0.199192002, 0.056736600},
    {-0.197550997, 0.064455599},
    {-0.195521995, 0.072169296},
    {-0.193124995, 0.079717703},
    {-0.190312997, 0.087211996},
    {-0.187123999, 0.094522998},
    {-0.183551997, 0.101654999},
    {-0.179591000, 0.108609997},
    {-0.175293997, 0.115396999},
    {-0.170816004, 0.122098997},
    {-0.166314006, 0.128750995},
    {-0.157168001, 0.141717002},
    {-0.152318999, 0.148013994},
    {-0.147204995, 0.154136002},
    {-0.141824007, 0.160023004},
    {-0.130223006, 0.170965001},
    {-0.124036998, 0.175954998},
    {-0.110849001, 0.184971005},
    {-0.096771099, 0.192582995},
    {-0.089476600, 0.196079001},
    {-0.082210302, 0.199503005},
    {-0.074938603, 0.202848002},
    {-0.067594700, 0.206031993},
    {-0.060205400, 0.208958998},
    {-0.052734699, 0.211587995},
    {-0.037450001, 0.215834007},
    {-0.029628800, 0.217398003},
    {-0.021765500, 0.218555003},
    {-0.013880700, 0.219300002},
    {0.000993459, 0.219565004},
    {0.006454560, 0.219294995},
    {0.011710800, 0.218907997},
    {0.017074799, 0.218454003},
    {0.028533000, 0.217482999},
    {0.034714900, 0.217003003},
    {0.041152701, 0.216544002},
    {0.047846500, 0.216108993},
    {0.061858501, 0.215290993},
    {0.069062702, 0.214894995},
    {0.076407798, 0.214491993},
    {0.083893098, 0.214073002},
    {0.091405503, 0.213633001},
    {0.099000402, 0.213156998},
    {0.106592998, 0.212641999},
    {0.114238001, 0.212077007},
    {0.121878996, 0.211457998},
    {0.129571006, 0.210773006},
    {0.137256995, 0.210023001},
    {0.144907996, 0.209208995},
    {0.160214007, 0.207365006},
    {0.167867005, 0.206325993},
    {0.175562993, 0.205197006},
    {0.183191001, 0.203989998},
    {0.198429003, 0.201307997},
    {0.205980003, 0.199829996},
    {0.213510007, 0.198248997},
    {0.221045002, 0.196557000},
    {0.228526995, 0.194760993},
    {0.235926002, 0.192863002},
    {0.243319005, 0.190834999},
    {0.253850013, 0.187686995},
    {0.256561011, 0.186820999},
    {0.258168012, 0.186294004},
    {0.259023994, 0.186009005},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
    {0.259398013, 0.185883000},
};

#endif // VARIABLES_H