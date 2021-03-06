#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
#define BAT_NTC_10 1
#define BAT_NTC_47 0
#define BAT_NTC_100 0

#ifdef CONFIG_MTK_PMIC_MT6397
#define RBAT_PULL_UP_R             24000
#define RBAT_PULL_DOWN_R           100000000
#define RBAT_PULL_UP_VOLT          1200

#else

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             16900
#define RBAT_PULL_DOWN_R           30000
#endif
#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900
#define RBAT_PULL_DOWN_R           100000
#endif
#if (BAT_NTC_100 == 1)
#define RBAT_PULL_UP_R             24000
#define RBAT_PULL_DOWN_R           100000000
#endif
#define RBAT_PULL_UP_VOLT          1800
#endif


// ============================================================
// ENUM
// ============================================================

// ============================================================
// structure
// ============================================================

// ============================================================
// typedef
// ============================================================
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// ============================================================
// External Variables
// ============================================================

// ============================================================
// External function
// ============================================================

// ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================
#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[2][17] = 
{
    {  // [0][]
        {-20,69709},
        {-15,54633},
        {-10,43155},
        { -5,34343},
        {  0,27523},
        {  5,22206},
        { 10,18031},
        { 15,14730},
        { 20,12103},
        { 25,10000},
        { 30,8306},
        { 35,6935},
        { 40,5818},
        { 45,4904},
        { 50,4152},
        { 55,3531},
        { 60,3016}
    },
    {//[1][]
        {-20,69709},
        {-15,54633},
        {-10,43155},
        { -5,34343},
        {  0,27523},
        {  5,22206},
        { 10,18031},
        { 15,14730},
        { 20,12103},
        { 25,10000},
        { 30,8306},
        { 35,6935},
        { 40,5818},
        { 45,4904},
        { 50,4152},
        { 55,3531},
        { 60,3016}
    }
};
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[2][17] = 
{
	{// [0][]
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}        
    },
{//[1][]
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}        
    }

};
#endif

/*battery fuel gauge para gen
Now we using MTK default behavior
*/


//  The following data is for temperature -10 C
//  Total data num: 75
// T0 -10C{DOD,OCV} 
 BATTERY_PROFILE_STRUC battery_profile_t0[2][75] =
{
 {//[0][]  yi gong 1900
    {0,4333}, //0
    {2,4308}, //
    {3,4287}, //3//
    {5,4269}, //5
    {6,4249}, //6
    {8,4232}, //8
    {9,4214}, //9
    {11,4197}, //11
    {13,4180}, //13
    {14,4163}, //14
    {16,4145}, //16
    {17,4129}, //17
    {19,4113}, //19
    {20,4096}, //20
    {22,4080}, //22
    {24,4064}, //24
    {25,4048}, //25
    {27,4031}, //27
    {28,4012}, //28
    {30,3993}, //30
    {31,3976}, //31
    {33,3962}, //33
    {34,3948}, //34
    {36,3934}, //36
    {38,3921}, //38
    {39,3909}, //39
    {41,3897}, //41
    {42,3885}, //42
    {44,3875}, //44
    {45,3865}, //45
    {47,3856}, //47
    {49,3847}, //49
    {50,3839}, //50
    {52,3831}, //52
    {53,3824}, //53
    {55,3816}, //55
    {56,3811}, //56
    {58,3804}, //58
    {59,3799}, //59
    {61,3793}, //61
    {62,3788}, //62
    {64,3783}, //64
    {66,3778}, //66
    {67,3773}, //67
    {69,3770}, //69
    {70,3766}, //70
    {72,3763}, //72
    {73,3758}, //73
    {75,3754}, //75
    {76,3748}, //76
    {78,3741}, //78
    {79,3733}, //79
    {81,3724}, //81
    {83,3714}, //83
    {84,3705}, //84
    {86,3699}, //86
    {87,3693}, //87
    {89,3688}, //89
    {90,3683}, //90
    {92,3678}, //92
    {93,3668}, //93
    {95,3641}, //95
    {96,3596}, //96
    {97,3560}, //97
    {98,3549}, //98
    {98,3543}, //98
    {98,3538}, //98
    {99,3533}, //99
    {99,3527}, //99
    {99,3522}, //99
    {99,3516}, //99
    {100,3510}, //100
    {100,3504}, //100
    {100,3498}, //100
    {100,3400} //100

 	},
	{   ////[0][]  er gong 1900
	 	 {0,4335}, //0
		 {3,4300}, //
		 {4,4277}, //
		 {6,4256}, //6
		 {7,4238}, //7
		 {9,4221}, //9
		 {10,4203}, //10
		 {12,4187}, //12
		 {13,4170}, //13
		 {15,4154}, //15
		 {16,4138}, //16
		 {18,4121}, //18
		 {19,4108}, //19
		 {21,4094}, //21
		 {22,4080}, //22
		 {24,4062}, //24
		 {25,4038}, //25
		 {27,4010}, //27
		 {28,3988}, //28
		 {29,3969}, //29
		 {31,3955}, //31
		 {32,3942}, //32
		 {34,3932}, //34
		 {35,3923}, //35
		 {37,3915}, //37
		 {38,3907}, //38
		 {40,3898}, //40
		 {41,3889}, //41
		 {43,3880}, //43
		 {44,3871}, //44
		 {46,3863}, //46
		 {47,3855}, //47
		 {49,3848}, //49
		 {50,3840}, //50
		 {52,3833}, //52
		 {53,3827}, //53
		 {55,3821}, //55
		 {56,3814}, //56
		 {57,3809}, //57
		 {59,3804}, //59
		 {60,3800}, //60
		 {62,3796}, //62
		 {63,3793}, //63
		 {65,3789}, //65
		 {66,3786}, //66
		 {68,3783}, //68
		 {69,3780}, //69
		 {71,3777}, //71
		 {72,3774}, //72
		 {74,3770}, //74
		 {75,3766}, //75
		 {77,3762}, //77
		 {78,3758}, //78
		 {80,3753}, //80
		 {81,3749}, //81
		 {81,3743}, //81
		 {83,3738}, //83
		 {84,3731}, //84
		 {85,3726}, //85
		 {87,3720}, //87
		 {88,3715}, //88
		 {90,3709}, //90
		 {91,3702}, //91
		 {93,3695}, //93
		 {94,3685}, //94
		 {96,3669}, //96
		 {97,3645}, //97
		 {99,3611}, //99
		 {100,3564}, //100
		 {100,3400}, //100
		 {100,3400}, //100
		 {100,3400}, //100
		 {100,3400}, //100
		 {100,3400}, //100
		 {100,3400} //100
		 }

};

//  Total data num: 75
// T0 r_profile N10C<Rbat, Battery_Voltage> 
 R_PROFILE_STRUC r_profile_t0[2][75] =
{
	{
    {482,4333}, //0
    {482,4308}, //
    {480,4287}, //3//
    {487,4269}, //5
    {478,4249}, //6
    {483,4232}, //8
    {485,4214}, //9
    {488,4197}, //11
    {490,4180}, //13
    {495,4163}, //14
    {490,4145}, //16
    {493,4129}, //17
    {495,4113}, //19
    {495,4096}, //20
    {495,4080}, //22
    {495,4064}, //24
    {493,4048}, //25
    {492,4031}, //27
    {488,4012}, //28
    {480,3993}, //30
    {480,3976}, //31
    {480,3962}, //33
    {477,3948}, //34
    {473,3934}, //36
    {470,3921}, //38
    {468,3909}, //39
    {467,3897}, //41
    {465,3885}, //42
    {468,3875}, //44
    {467,3865}, //45
    {468,3856}, //47
    {470,3847}, //49
    {472,3839}, //50
    {472,3831}, //52
    {477,3824}, //53
    {473,3816}, //55
    {477,3811}, //56
    {480,3804}, //58
    {485,3799}, //59
    {488,3793}, //61
    {488,3788}, //62
    {497,3783}, //64
    {497,3778}, //66
    {500,3773}, //67
    {505,3770}, //69
    {507,3766}, //70
    {517,3763}, //72
    {518,3758}, //73
    {528,3754}, //75
    {533,3748}, //76
    {542,3741}, //78
    {550,3733}, //79
    {560,3724}, //81
    {567,3714}, //83
    {578,3705}, //84
    {592,3699}, //86
    {610,3693}, //87
    {632,3688}, //89
    {662,3683}, //90
    {702,3678}, //92
    {750,3668}, //93
    {795,3641}, //95
    {862,3596}, //96
    {933,3560}, //97
    {915,3549}, //98
    {905,3543}, //98
    {897,3538}, //98
    {888,3533}, //99
    {878,3527}, //99
    {870,3522}, //99
    {860,3516}, //99
    {850,3510}, //100
    {840,3504}, //100
    {830,3498}, //100
    {820,3400} //100

 	},
	{
    {1246,4335}, //0
    {1246,4300}, 
    {1240,4277}, 
    {1223,4256}, //6
    {1212,4238}, //7
    {1197,4221}, //9
    {1174,4203}, //10
    {1149,4187}, //12
    {1136,4170}, //13
    {1119,4154}, //15
    {1113,4138}, //16
    {1102,4121}, //18
    {1107,4108}, //19
    {1099,4094}, //21
    {1096,4080}, //22
    {1073,4062}, //24
    {1039,4038}, //25
    {1003,4010}, //27
    {979,3988}, //28
    {962,3969}, //29
    {954,3955}, //31
    {946,3942}, //32
    {944,3932}, //34
    {939,3923}, //35
    {942,3915}, //37
    {943,3907}, //38
    {938,3898}, //40
    {936,3889}, //41
    {936,3880}, //43
    {935,3871}, //44
    {932,3863}, //46
    {932,3855}, //47
    {938,3848}, //49
    {939,3840}, //50
    {939,3833}, //52
    {946,3827}, //53
    {947,3821}, //55
    {947,3814}, //56
    {952,3809}, //57
    {958,3804}, //59
    {967,3800}, //60
    {974,3796}, //62
    {979,3793}, //63
    {989,3789}, //65
    {1001,3786}, //66
    {1011,3783}, //68
    {1026,3780}, //69
    {1040,3777}, //71
    {1056,3774}, //72
    {1071,3770}, //74
    {1090,3766}, //75
    {1110,3762}, //77
    {1133,3758}, //78
    {1158,3753}, //80
    {1187,3749}, //81
    {1212,3743}, //81
    {1245,3738}, //83
    {1283,3731}, //84
    {1321,3726}, //85
    {1369,3720}, //87
    {1418,3715}, //88
    {1479,3709}, //90
    {1539,3702}, //91
    {1603,3695}, //93
    {1668,3685}, //94
    {1728,3669}, //96
    {1789,3645}, //97
    {1849,3611}, //99
    {1928,3564}, //100
    {2027,3400}, //100
    {2027,3400}, //100
    {2027,3400}, //100
    {2027,3400}, //100
    {2027,3400}, //100
    {2027,3400} //100
	}

 };



//  The following data is for temperature 0 C
//  Total data num: 75
// T1 0C{DOD,OCV} 
 BATTERY_PROFILE_STRUC battery_profile_t1[2][75] =
{
	{
    {0,4337}, //0
    {2,4315}, //
    {3,4296}, //3//
    {5,4279}, //5
    {6,4261}, //6
    {8,4243}, //8
    {9,4226}, //9
    {11,4209}, //11
    {12,4192}, //12
    {14,4175}, //14
    {15,4158}, //15
    {17,4142}, //17
    {19,4125}, //19
    {20,4110}, //20
    {22,4094}, //22
    {23,4078}, //23
    {25,4064}, //25
    {26,4048}, //26
    {28,4031}, //28
    {29,4015}, //29
    {31,3998}, //31
    {33,3982}, //33
    {34,3969}, //34
    {36,3954}, //36
    {37,3940}, //37
    {39,3926}, //39
    {40,3913}, //40
    {42,3900}, //42
    {43,3889}, //43
    {45,3878}, //45
    {46,3868}, //46
    {48,3860}, //48
    {50,3851}, //50
    {51,3844}, //51
    {53,3837}, //53
    {54,3829}, //54
    {56,3823}, //56
    {57,3817}, //57
    {59,3812}, //59
    {60,3807}, //60
    {62,3801}, //62
    {63,3796}, //63
    {65,3792}, //65
    {67,3787}, //67
    {68,3784}, //68
    {70,3779}, //70
    {71,3776}, //71
    {73,3773}, //73
    {74,3769}, //74
    {76,3765}, //76
    {77,3759}, //77
    {79,3752}, //79
    {81,3742}, //81
    {82,3732}, //82
    {84,3720}, //84
    {85,3711}, //85
    {87,3704}, //87
    {88,3700}, //88
    {90,3697}, //90
    {91,3692}, //91
    {93,3688}, //93
    {94,3680}, //94
    {96,3655}, //96
    {98,3606}, //98
    {99,3545}, //99
    {100,3448}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400} //100
	},
	{
    {0,4340}, //0
    {1,4319}, 
    {3,4300}, 
    {4,4282}, 
    {6,4265}, //6
    {7,4250}, //7
    {9,4235}, //9
    {10,4219}, //10
    {11,4204}, //11
    {13,4190}, //13
    {14,4174}, //14
    {16,4160}, //16
    {17,4144}, //17
    {18,4129}, //18
    {20,4114}, //20
    {21,4101}, //21
    {23,4090}, //23
    {24,4082}, //24
    {26,4069}, //26
    {27,4046}, //27
    {28,4015}, //28
    {30,3989}, //30
    {31,3972}, //31
    {33,3956}, //33
    {34,3944}, //34
    {35,3934}, //35
    {37,3925}, //37
    {38,3917}, //38
    {40,3909}, //40
    {41,3899}, //41
    {43,3889}, //43
    {44,3880}, //44
    {45,3871}, //45
    {47,3863}, //47
    {48,3854}, //48
    {50,3847}, //50
    {51,3840}, //51
    {53,3834}, //53
    {54,3828}, //54
    {55,3821}, //55
    {57,3816}, //57
    {58,3811}, //58
    {60,3806}, //60
    {61,3801}, //61
    {62,3797}, //62
    {64,3793}, //64
    {65,3789}, //65
    {67,3785}, //67
    {68,3782}, //68
    {70,3780}, //70
    {71,3777}, //71
    {72,3774}, //72
    {74,3771}, //74
    {75,3768}, //75
    {77,3764}, //77
    {78,3760}, //78
    {79,3755}, //79
    {81,3749}, //81
    {82,3742}, //82
    {84,3737}, //84
    {85,3730}, //85
    {87,3722}, //87
    {88,3713}, //88
    {89,3706}, //89
    {91,3703}, //91
    {92,3700}, //92
    {94,3695}, //94
    {95,3681}, //95
    {97,3641}, //97
    {98,3578}, //98
    {99,3482}, //99
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400} //100
	}
};
//  Total data num: 75
// T1 r_profile 0C<Rbat, Battery_Voltage> 
 R_PROFILE_STRUC r_profile_t1[2][75] =
{
	{
    {298,4337}, //0
    {298,4315}, //
    {302,4296}, //3//
    {307,4279}, //5
    {307,4261}, //6
    {310,4243}, //8
    {312,4226}, //9
    {313,4209}, //11
    {315,4192}, //12
    {318,4175}, //14
    {325,4158}, //15
    {322,4142}, //17
    {323,4125}, //19
    {328,4110}, //20
    {330,4094}, //22
    {330,4078}, //23
    {335,4064}, //25
    {333,4048}, //26
    {333,4031}, //28
    {335,4015}, //29
    {332,3998}, //31
    {327,3982}, //33
    {327,3969}, //34
    {323,3954}, //36
    {318,3940}, //37
    {315,3926}, //39
    {312,3913}, //40
    {308,3900}, //42
    {307,3889}, //43
    {307,3878}, //45
    {305,3868}, //46
    {308,3860}, //48
    {307,3851}, //50
    {310,3844}, //51
    {312,3837}, //53
    {310,3829}, //54
    {312,3823}, //56
    {315,3817}, //57
    {317,3812}, //59
    {318,3807}, //60
    {320,3801}, //62
    {320,3796}, //63
    {323,3792}, //65
    {323,3787}, //67
    {325,3784}, //68
    {323,3779}, //70
    {323,3776}, //71
    {325,3773}, //73
    {327,3769}, //74
    {330,3765}, //76
    {332,3759}, //77
    {333,3752}, //79
    {335,3742}, //81
    {338,3732}, //82
    {345,3720}, //84
    {343,3711}, //85
    {345,3704}, //87
    {357,3700}, //88
    {370,3697}, //90
    {387,3692}, //91
    {415,3688}, //93
    {452,3680}, //94
    {492,3655}, //96
    {543,3606}, //98
    {618,3545}, //99
    {723,3448}, //100
    {653,3400}, //100
    {653,3400}, //100
    {653,3400}, //100
    {653,3400}, //100
    {653,3400}, //100
    {653,3400}, //100
    {653,3400}, //100
    {653,3400}, //100
    {653,3400} //100
	},
	{
    
		{510,4340}, //0
		{510,4319}, 
		{505,4300}, 
		{501,4282}, 
		{496,4265}, //6
		{493,4250}, //7
		{493,4235}, //9
		{487,4219}, //10
		{479,4204}, //11
		{479,4190}, //13
		{471,4174}, //14
		{470,4160}, //16
		{467,4144}, //17
		{465,4129}, //18
		{460,4114}, //20
		{459,4101}, //21
		{460,4090}, //23
		{471,4082}, //24
		{475,4069}, //26
		{459,4046}, //27
		{437,4015}, //28
		{423,3989}, //30
		{417,3972}, //31
		{415,3956}, //33
		{406,3944}, //34
		{403,3934}, //35
		{403,3925}, //37
		{400,3917}, //38
		{395,3909}, //40
		{395,3899}, //41
		{390,3889}, //43
		{388,3880}, //44
		{389,3871}, //45
		{391,3863}, //47
		{391,3854}, //48
		{392,3847}, //50
		{395,3840}, //51
		{398,3834}, //53
		{402,3828}, //54
		{402,3821}, //55
		{405,3816}, //57
		{409,3811}, //58
		{409,3806}, //60
		{417,3801}, //61
		{418,3797}, //62
		{422,3793}, //64
		{426,3789}, //65
		{425,3785}, //67
		{428,3782}, //68
		{429,3780}, //70
		{429,3777}, //71
		{443,3774}, //72
		{446,3771}, //74
		{450,3768}, //75
		{456,3764}, //77
		{461,3760}, //78
		{467,3755}, //79
		{474,3749}, //81
		{481,3742}, //82
		{484,3737}, //84
		{496,3730}, //85
		{508,3722}, //87
		{515,3713}, //88
		{536,3706}, //89
		{558,3703}, //91
		{591,3700}, //92
		{636,3695}, //94
		{687,3681}, //95
		{721,3641}, //97
		{789,3578}, //98
		{907,3482}, //99
		{1147,3400}, //100
	    {1147,3400}, //100
	    {1147,3400}, //100
	    {1147,3400} //100
    
    }
};

//  The following data is for temperature 25 C
//  Total data num: 75
// T2 25C{DOD,OCV} 
 BATTERY_PROFILE_STRUC battery_profile_t2[2][75] =
{
	{
    {0,4333}, //0
    {2,4312}, //
    {3,4294}, //3//
    {5,4276}, //5
    {6,4258}, //6
    {8,4241}, //8
    {9,4224}, //9
    {11,4207}, //11
    {12,4190}, //12
    {14,4174}, //14
    {15,4157}, //15
    {17,4141}, //17
    {18,4126}, //18
    {20,4110}, //20
    {21,4095}, //21
    {23,4080}, //23
    {24,4065}, //24
    {26,4051}, //26
    {27,4037}, //27
    {29,4022}, //29
    {30,4009}, //30
    {32,3996}, //32
    {33,3984}, //33
    {35,3972}, //35
    {36,3963}, //36
    {38,3951}, //38
    {39,3940}, //39
    {41,3928}, //41
    {42,3913}, //42
    {44,3897}, //44
    {45,3881}, //45
    {47,3867}, //47
    {48,3856}, //48
    {50,3846}, //50
    {51,3838}, //51
    {53,3830}, //53
    {54,3824}, //54
    {56,3818}, //56
    {57,3811}, //57
    {59,3806}, //59
    {60,3801}, //60
    {62,3797}, //62
    {63,3792}, //63
    {65,3787}, //65
    {66,3784}, //66
    {68,3780}, //68
    {69,3776}, //69
    {71,3773}, //71
    {72,3771}, //72
    {74,3768}, //74
    {75,3764}, //75
    {77,3760}, //77
    {78,3753}, //78
    {80,3748}, //80
    {81,3741}, //81
    {83,3733}, //83
    {84,3721}, //84
    {86,3710}, //86
    {87,3695}, //87
    {89,3687}, //89
    {90,3684}, //90
    {92,3682}, //92
    {93,3680}, //93
    {95,3676}, //95
    {96,3663}, //96
    {98,3605}, //98
    {99,3513}, //99
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400} //100
	},
	{
    {0,4333}, //0
    {1,4314}, 
    {3,4295}, 
    {4,4279}, 
    {6,4263}, //6
    {7,4247}, //7
    {9,4231}, //9
    {10,4215}, //10
    {11,4200}, //11
    {13,4184}, //13
    {14,4169}, //14
    {16,4154}, //16
    {17,4138}, //17
    {18,4123}, //18
    {20,4108}, //20
    {21,4094}, //21
    {23,4080}, //23
    {24,4072}, //24
    {26,4063}, //26
    {27,4043}, //27
    {28,4018}, //28
    {30,3998}, //30
    {31,3984}, //31
    {33,3974}, //33
    {34,3969}, //34
    {36,3961}, //36
    {37,3947}, //37
    {38,3931}, //38
    {40,3916}, //40
    {41,3901}, //41
    {43,3891}, //43
    {44,3880}, //44
    {46,3872}, //46
    {47,3863}, //47
    {48,3854}, //48
    {50,3847}, //50
    {51,3841}, //51
    {53,3834}, //53
    {54,3828}, //54
    {55,3823}, //55
    {57,3816}, //57
    {58,3811}, //58
    {60,3806}, //60
    {61,3802}, //61
    {63,3798}, //63
    {64,3794}, //64
    {65,3790}, //65
    {67,3785}, //67
    {68,3781}, //68
    {70,3776}, //70
    {71,3770}, //71
    {73,3765}, //73
    {74,3759}, //74
    {75,3754}, //75
    {77,3749}, //77
    {78,3745}, //78
    {80,3741}, //80
    {81,3736}, //81
    {83,3730}, //83
    {84,3723}, //84
    {85,3716}, //85
    {87,3708}, //87
    {88,3697}, //88
    {90,3692}, //90
    {91,3690}, //91
    {92,3688}, //92
    {94,3683}, //94
    {95,3668}, //95
    {97,3622}, //97
    {98,3554}, //98
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400} //100
	}
};
//  Total data num: 75
// T3 r_profile 25C<Rbat, Battery_Voltage> 
 R_PROFILE_STRUC r_profile_t2[2][75] =
{
	{
    {87,4333}, //0
    {133,4312}, //
    {137,4294}, //3//
    {137,4276}, //5
    {138,4258}, //6
    {140,4241}, //8
    {142,4224}, //9
    {142,4207}, //11
    {143,4190}, //12
    {145,4174}, //14
    {147,4157}, //15
    {148,4141}, //17
    {152,4126}, //18
    {153,4110}, //20
    {155,4095}, //21
    {157,4080}, //23
    {158,4065}, //24
    {157,4051}, //26
    {158,4037}, //27
    {162,4022}, //29
    {163,4009}, //30
    {165,3996}, //32
    {168,3984}, //33
    {170,3972}, //35
    {177,3963}, //36
    {177,3951}, //38
    {178,3940}, //39
    {177,3928}, //41
    {170,3913}, //42
    {160,3897}, //44
    {150,3881}, //45
    {145,3867}, //47
    {142,3856}, //48
    {140,3846}, //50
    {140,3838}, //51
    {140,3830}, //53
    {143,3824}, //54
    {143,3818}, //56
    {143,3811}, //57
    {145,3806}, //59
    {147,3801}, //60
    {148,3797}, //62
    {150,3792}, //63
    {150,3787}, //65
    {152,3784}, //66
    {152,3780}, //68
    {152,3776}, //69
    {152,3773}, //71
    {155,3771}, //72
    {155,3768}, //74
    {153,3764}, //75
    {152,3760}, //77
    {147,3753}, //78
    {148,3748}, //80
    {147,3741}, //81
    {150,3733}, //83
    {148,3721}, //84
    {150,3710}, //86
    {148,3695}, //87
    {145,3687}, //89
    {147,3684}, //90
    {150,3682}, //92
    {158,3680}, //93
    {167,3676}, //95
    {182,3663}, //96
    {173,3605}, //98
    {187,3513}, //99
    {213,3400}, //100
    {213,3400}, //100
    {213,3400}, //100
    {213,3400}, //100
    {213,3400}, //100
    {213,3400}, //100
    {213,3400}, //100
    {213,3400} //100

 },
	{
	{154,4333}, //0
    {154,4314}, 
    {151,4295}, 
    {152,4279},
    {152,4263}, //6
    {150,4247}, //7
    {150,4231}, //9
    {151,4215}, //10
    {149,4200}, //11
    {151,4184}, //13
    {152,4169}, //14
    {150,4154}, //16
    {152,4138}, //17
    {154,4123}, //18
    {154,4108}, //20
    {155,4094}, //21
    {157,4080}, //23
    {164,4072}, //24
    {171,4063}, //26
    {163,4043}, //27
    {157,4018}, //28
    {158,3998}, //30
    {161,3984}, //31
    {164,3974}, //33
    {171,3969}, //34
    {171,3961}, //36
    {166,3947}, //37
    {163,3931}, //38
    {158,3916}, //40
    {152,3901}, //41
    {152,3891}, //43
    {152,3880}, //44
    {152,3872}, //46
    {150,3863}, //47
    {147,3854}, //48
    {149,3847}, //50
    {152,3841}, //51
    {153,3834}, //53
    {153,3828}, //54
    {155,3823}, //55
    {154,3816}, //57
    {155,3811}, //58
    {157,3806}, //60
    {158,3802}, //61
    {160,3798}, //63
    {161,3794}, //64
    {163,3790}, //65
    {163,3785}, //67
    {163,3781}, //68
    {161,3776}, //70
    {157,3770}, //71
    {155,3765}, //73
    {154,3759}, //74
    {155,3754}, //75
    {153,3749}, //77
    {155,3745}, //78
    {157,3741}, //80
    {157,3736}, //81
    {158,3730}, //83
    {158,3723}, //84
    {158,3716}, //85
    {160,3708}, //87
    {155,3697}, //88
    {153,3692}, //90
    {155,3690}, //91
    {160,3688}, //92
    {164,3683}, //94
    {166,3668}, //95
    {164,3622}, //97
    {174,3554}, //98
    {189,3400}, //100
    {189,3400}, //100
    {189,3400}, //100
    {189,3400}, //100
    {189,3400} //100
	
	 }
};


//  The following data is for temperature 55 C
//  Total data num: 75
// T3 50C:{DOD,OCV} 
 BATTERY_PROFILE_STRUC battery_profile_t3[2][75] =
{
	{
    {0,4323}, //0
    {2,4301}, //
    {3,4282}, //3//
    {5,4265}, //5
    {6,4247}, //6
    {8,4230}, //8
    {9,4213}, //9
    {11,4195}, //11
    {12,4179}, //12
    {14,4163}, //14
    {15,4147}, //15
    {17,4130}, //17
    {18,4115}, //18
    {20,4098}, //20
    {21,4083}, //21
    {23,4067}, //23
    {24,4052}, //24
    {26,4038}, //26
    {27,4024}, //27
    {29,4010}, //29
    {30,3997}, //30
    {32,3984}, //32
    {33,3971}, //33
    {35,3962}, //35
    {36,3950}, //36
    {38,3939}, //38
    {39,3929}, //39
    {41,3918}, //41
    {42,3906}, //42
    {44,3892}, //44
    {45,3874}, //45
    {47,3857}, //47
    {49,3846}, //49
    {50,3837}, //50
    {52,3828}, //52
    {53,3821}, //53
    {55,3814}, //55
    {56,3808}, //56
    {58,3802}, //58
    {59,3796}, //59
    {61,3790}, //61
    {62,3786}, //62
    {64,3780}, //64
    {65,3776}, //65
    {67,3772}, //67
    {68,3768}, //68
    {70,3765}, //70
    {71,3759}, //71
    {73,3749}, //73
    {74,3741}, //74
    {76,3737}, //76
    {77,3731}, //77
    {79,3725}, //79
    {80,3718}, //80
    {82,3711}, //82
    {83,3703}, //83
    {85,3689}, //85
    {86,3676}, //86
    {88,3663}, //88
    {89,3659}, //89
    {91,3658}, //91
    {92,3656}, //92
    {94,3651}, //94
    {95,3643}, //95
    {97,3597}, //97
    {98,3521}, //98
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400} //100

 	},
 	{
    {0,4338}, //0
    {1,4307}, 
    {3,4291}, 
    {4,4275}, 
    {6,4259}, //6
    {7,4242}, //7
    {9,4227}, //9
    {10,4211}, //10
    {11,4195}, //11
    {13,4180}, //13
    {14,4164}, //14
    {16,4149}, //16
    {17,4134}, //17
    {19,4119}, //19
    {20,4105}, //20
    {21,4090}, //21
    {23,4075}, //23
    {24,4061}, //24
    {26,4049}, //26
    {27,4038}, //27
    {29,4018}, //29
    {30,4004}, //30
    {31,3996}, //31
    {33,3984}, //33
    {34,3972}, //34
    {36,3960}, //36
    {37,3945}, //37
    {39,3928}, //39
    {40,3911}, //40
    {41,3898}, //41
    {43,3888}, //43
    {44,3878}, //44
    {46,3869}, //46
    {47,3860}, //47
    {49,3852}, //49
    {50,3845}, //50
    {52,3838}, //52
    {53,3832}, //53
    {54,3826}, //54
    {56,3819}, //56
    {57,3813}, //57
    {59,3808}, //59
    {60,3803}, //60
    {62,3798}, //62
    {63,3793}, //63
    {64,3789}, //64
    {66,3785}, //66
    {67,3780}, //67
    {69,3774}, //69
    {70,3764}, //70
    {72,3754}, //72
    {73,3748}, //73
    {74,3741}, //74
    {76,3734}, //76
    {77,3730}, //77
    {79,3725}, //79
    {80,3721}, //80
    {82,3716}, //82
    {83,3709}, //83
    {84,3702}, //84
    {86,3696}, //86
    {87,3688}, //87
    {89,3679}, //89
    {90,3676}, //90
    {92,3675}, //92
    {93,3672}, //93
    {94,3666}, //94
    {96,3636}, //96
    {97,3581}, //97
    {99,3506}, //99
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400}, //100
    {100,3400} //100
	}
};
//  Total data num: 75
// T3 r_profile 50C<Rbat, Battery_Voltage> 
 R_PROFILE_STRUC r_profile_t3[2][75] =
{
	{
    {123,4323}, //0
    {123,4301},// 1
    {120,4282},// 2
    {123,4265}, //5
    {123,4247}, //6
    {123,4230}, //8
    {123,4213},//9
    {122,4195}, //11
    {123,4179}, //12
    {125,4163}, //14
    {128,4147}, //15
    {127,4130}, //17
    {130,4115}, //18
    {130,4098}, //20
    {128,4083}, //21
    {132,4067}, //23
    {130,4052}, //24
    {130,4038}, //26
    {130,4024}, //27
    {130,4010}, //29
    {132,3997}, //30
    {132,3984}, //32
    {132,3971}, //33
    {140,3962}, //35
    {145,3950}, //36
    {147,3939}, //38
    {150,3929}, //39
    {153,3918}, //41
    {155,3906}, //42
    {152,3892}, //44
    {142,3874}, //45
    {130,3857}, //47
    {127,3846}, //49
    {127,3837}, //50
    {123,3828}, //52
    {123,3821}, //53
    {123,3814}, //55
    {128,3808}, //56
    {128,3802}, //58
    {130,3796}, //59
    {128,3790}, //61
    {133,3786}, //62
    {130,3780}, //64
    {135,3776}, //65
    {135,3772}, //67
    {135,3768}, //68
    {138,3765}, //70
    {137,3759}, //71
    {128,3749}, //73
    {127,3741}, //74
    {130,3737}, //76
    {127,3731}, //77
    {128,3725}, //79
    {127,3718}, //80
    {128,3711}, //82
    {127,3703}, //83
    {127,3689}, //85
    {127,3676}, //86
    {127,3663}, //88
    {123,3659}, //89
    {125,3658}, //91
    {130,3656}, //92
    {130,3651}, //94
    {138,3643}, //95
    {128,3597}, //97
    {142,3521}, //98
    {148,3400}, //100
    {148,3400}, //100
    {148,3400}, //100
    {148,3400}, //100
    {148,3400}, //100
    {148,3400}, //100
    {148,3400}, //100
    {148,3400}, //100
    {148,3400} //100
	},
	{
	{118,4338}, //0
    {118,4307}, 
    {118,4291}, 
    {119,4275}, 
    {119,4259}, //6
    {120,4242}, //7
    {120,4227}, //9
    {122,4211}, //10
    {121,4195}, //11
    {121,4180}, //13
    {121,4164}, //14
    {121,4149}, //16
    {119,4134}, //17
    {121,4119}, //19
    {122,4105}, //20
    {121,4090}, //21
    {123,4075}, //23
    {124,4061}, //24
    {126,4049}, //26
    {132,4038}, //27
    {126,4018}, //29
    {130,4004}, //30
    {133,3996}, //31
    {132,3984}, //33
    {133,3972}, //34
    {136,3960}, //36
    {134,3945}, //37
    {127,3928}, //39
    {122,3911}, //40
    {119,3898}, //41
    {119,3888}, //43
    {120,3878}, //44
    {121,3869}, //46
    {119,3860}, //47
    {120,3852}, //49
    {121,3845}, //50
    {119,3838}, //52
    {119,3832}, //53
    {122,3826}, //54
    {120,3819}, //56
    {121,3813}, //57
    {121,3808}, //59
    {123,3803}, //60
    {124,3798}, //62
    {123,3793}, //63
    {126,3789}, //64
    {129,3785}, //66
    {129,3780}, //67
    {127,3774}, //69
    {121,3764}, //70
    {120,3754}, //72
    {120,3748}, //73
    {123,3741}, //74
    {119,3734}, //76
    {121,3730}, //77
    {121,3725}, //79
    {121,3721}, //80
    {121,3716}, //82
    {121,3709}, //83
    {121,3702}, //84
    {123,3696}, //86
    {122,3688}, //87
    {121,3679}, //89
    {121,3676}, //90
    {124,3675}, //92
    {126,3672}, //93
    {130,3666}, //94
    {126,3636}, //96
    {129,3581}, //97
    {133,3506}, //99
    {143,3400}, //100
    {143,3400}, //100
    {143,3400}, //100
    {143,3400}, //100
    {143,3400} //100
	
	 }
};



//  The following data is for temperature empty C
//  Total data num: 75
//battery profile for actual temperature. The size should be the same as T1, T2 and T3
 BATTERY_PROFILE_STRUC battery_profile_temperature[75] =
 {
    {0,0}, //
    {0,0}, //
    {0,0}, //3//
    {0,0}, //4//
    {0,0}, //5
    {0,0}, //6
    {0,0}, //7
    {0,0}, //8
    {0,0}, //9
    {0,0}, //10
    {0,0}, //11
    {0,0}, //12
    {0,0}, //13
    {0,0}, //14
    {0,0}, //15
    {0,0}, //16
    {0,0}, //17
    {0,0}, //18
    {0,0}, //19
    {0,0}, //20
    {0,0}, //21
    {0,0}, //22
    {0,0}, //23
    {0,0}, //24
    {0,0}, //25
    {0,0}, //26
    {0,0}, //27
    {0,0}, //28
    {0,0}, //29
    {0,0}, //30
    {0,0}, //31
    {0,0}, //32
    {0,0}, //33
    {0,0}, //34
    {0,0}, //35
    {0,0}, //36
    {0,0}, //37
    {0,0}, //38
    {0,0}, //39
    {0,0}, //40
    {0,0}, //41
    {0,0}, //42
    {0,0}, //43
    {0,0}, //44
    {0,0}, //45
    {0,0}, //46
    {0,0}, //47
    {0,0}, //48
    {0,0}, //49
    {0,0}, //50
    {0,0}, //51
    {0,0}, //52
    {0,0}, //53
    {0,0}, //54
    {0,0}, //55
    {0,0}, //56
    {0,0}, //57
    {0,0}, //58
    {0,0}, //59
    {0,0}, //60
    {0,0}, //61
    {0,0}, //62
    {0,0}, //63
    {0,0}, //64
    {0,0}, //65
    {0,0}, //66
    {0,0}, //67
    {0,0}, //68
    {0,0}, //69
    {0,0}, //70
    {0,0}, //71
    {0,0}, //72
    {0,0}, //73
    {0,0}, //74
    {0,0} //75

 };
//  Total data num: 75
//r-table profile for actual temperature. The size should be the same as T1, T2 and T3
 R_PROFILE_STRUC r_profile_temperature[75] =
 {
    {0,0}, //
    {0,0}, //
    {0,0}, //
    {0,0}, //4//
    {0,0}, //5
    {0,0}, //6
    {0,0}, //7
    {0,0}, //8
    {0,0}, //9
    {0,0}, //10
    {0,0}, //11
    {0,0}, //12
    {0,0}, //13
    {0,0}, //14
    {0,0}, //15
    {0,0}, //16
    {0,0}, //17
    {0,0}, //18
    {0,0}, //19
    {0,0}, //20
    {0,0}, //21
    {0,0}, //22
    {0,0}, //23
    {0,0}, //24
    {0,0}, //25
    {0,0}, //26
    {0,0}, //27
    {0,0}, //28
    {0,0}, //29
    {0,0}, //30
    {0,0}, //31
    {0,0}, //32
    {0,0}, //33
    {0,0}, //34
    {0,0}, //35
    {0,0}, //36
    {0,0}, //37
    {0,0}, //38
    {0,0}, //39
    {0,0}, //40
    {0,0}, //41
    {0,0}, //42
    {0,0}, //43
    {0,0}, //44
    {0,0}, //45
    {0,0}, //46
    {0,0}, //47
    {0,0}, //48
    {0,0}, //49
    {0,0}, //50
    {0,0}, //51
    {0,0}, //52
    {0,0}, //53
    {0,0}, //54
    {0,0}, //55
    {0,0}, //56
    {0,0}, //57
    {0,0}, //58
    {0,0}, //59
    {0,0}, //60
    {0,0}, //61
    {0,0}, //62
    {0,0}, //63
    {0,0}, //64
    {0,0}, //65
    {0,0}, //66
    {0,0}, //67
    {0,0}, //68
    {0,0}, //69
    {0,0}, //70
    {0,0}, //71
    {0,0}, //72
    {0,0}, //73
    {0,0}, //74
    {0,0} //75

 };
// ============================================================
// function prototype
// ============================================================
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);

#endif	//#ifndef _CUST_BATTERY_METER_TABLE_H

