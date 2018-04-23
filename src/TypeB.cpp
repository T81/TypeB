// Type B Thermocouple library per ITS-90

// *** BSD License ***
// ------------------------------------------------------------------------------------------
//
// Author: T81
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this list of 
// conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice, this list 
// of conditions and the following disclaimer in the documentation and/or other materials 
// provided with the distribution.
//
// Neither the name of the MLG Properties, LLC nor the names of its contributors may be 
// used to endorse or promote products derived from this software without specific prior 
// written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// Original Author of Type K thermocouples library: Jim Gallt
// ------------------------------------------------------------------------------------------

#include "TypeB.h"
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif


// -------------------------------------
const int TypeB::nranges_inv = 2;  // number of mV ranges for inverse lookup
const int TypeB::ncoeff_inv = 9;  // number of coefficients for inverse lookup
const float TypeB::mv_min = 0.291;
const float TypeB::mv_max = 13.820;

// coefficients for inverse lookup (given mV, find C)
const double TypeB::coeff_inv[9][2] = {
	{  9.8423321E+01,  2.1315071E+02 },
	{  6.9971500E+02,  2.8510504E+02 },
	{ -8.4765304E+02, -5.2742887E+01 },
	{  1.0052644E+03,  9.9160804E+00 },
	{ -8.3345952E+02, -1.2965303E+00 },
	{  4.5508542E+02,  1.1195870E-01 },
	{ -1.5523037E+02, -6.0625199E-03 },		 
	{  2.9886750E+01,  1.8661696E-04 },
	{ -2.4742860E+00, -2.4878585E-06 }

};

// mV ranges for inverse lookup coefficients
const float TypeB::range_inv[2][2] = {
  { 0.291,  2.431 },
  { 2.431, 13.820 }
};

// coefficients for direct lookup (given C, find mV)
const double TypeB::coeff_dir[9][2] = {
	{  0.000000000000E+00, -0.389381686210E+01 },
	{ -0.246508183460E-03,  0.285717474700E-01 },
	{  0.590404211710E-05, -0.848851047850E-04 },
	{ -0.132579316360E-08,  0.157852801640E-06 },
	{  0.156682919010E-11, -0.168353448640E-09 },
	{ -0.169445292400E-14,  0.111097940130E-12 },
	{  0.629903470940E-18, -0.445154310330E-16 },
	{  0.000000000000E+00,  0.989756408210E-20 },
	{  0.000000000000E+00, -0.937913302890E-24 }
};

// ranges for direct lookup
const double TypeB::range_dir[2][2] = {
  { 0.000,    630.615 },
  { 630.615, 1820.000 }
};

const float TypeB::C_max = 1820.0;
const float TypeB::C_min = 0.0;

// -------------------------- constructor
TypeB::TypeB() {
  F_max = C_TO_F( C_max );
  F_min = C_TO_F( C_min );
}

// ------------------- given mv reading, returns absolute temp C
double TypeB::Temp_C( float mv ) {
  double x = 1.0;
  double sum = 0.0;
  int i,j,ind;
  ind = 0;
  if ( ! inrange_mV( mv ) ) return TC_RANGE_ERR;
  // first figure out which range of values
  for( j = 0; j < nranges_inv; j++ ) {
    if((mv >= range_inv[0][j]) && (mv <= range_inv[1][j]))
      ind = j;
  };
//  Serial.println(ind);
  for( i = 0; i < ncoeff_inv; i++ ) {
    sum += x * coeff_inv[i][ind];
    x *= mv;
  }
  return sum;  
}

// --------- given mv reading and ambient temp, returns compensated (true)
//           temperature at tip of sensor
double TypeB::Temp_C( float mv, float amb ) {
  float mv_amb;
  mv_amb = mV_C( amb );
  return Temp_C( mv + mv_amb );
};

// --------------------- returns compensated temperature in F units
double TypeB::Temp_F( float mv, float amb ) {
  return C_TO_F( Temp_C( mv, F_TO_C( amb ) ) );
};

// --------------------- returns absolute temperature in F units
double TypeB::Temp_F( float mv ) {
  float temp = Temp_C( mv );
  if( temp == TC_RANGE_ERR ) return TC_RANGE_ERR;
  return C_TO_F( temp );  
}

// --------------------- checks to make sure mv signal in range
boolean TypeB::inrange_mV( float mv ) {
  return ( mv >= mv_min ) & ( mv <= mv_max );
};

// ---------------------- checks to make sure temperature in range
boolean TypeB::inrange_C( float ambC ) {
  return ( ambC >= C_min ) & ( ambC <= C_max );
};

// ----------------------- checks to make sure temperature in range
boolean TypeB::inrange_F( float ambF ) {
  return ( ambF >= F_min ) & ( ambF <= F_max );
};

// ---------------- returns mV corresponding to temp reading
//                  used for cold junction compensation
double TypeB::mV_C( float ambC ) {
  double sum = 0.0;
  double x = 1.0;
  int i;
  if( !inrange_C( ambC ) ) return TC_RANGE_ERR;

  if( (ambC >= range_dir[0][0]) && ( ambC <= range_dir[1][0] ) ) {
    for( i = 0; i < 9; i++ ) {
      sum += x * coeff_dir[i][0];
      x *= ambC;
    } 
  }
  else {
    for( i = 0; i < 9; i++ ) {
      sum += x * coeff_dir[i][1];
      x *= ambC;    
    };
//    Serial.print( sum );
  };  
  return sum;
};

// -------------------- cold junction compensation in F units
double TypeB::mV_F( float ambF ) {
  if( inrange_F( ambF ) )
    return mV_C( F_TO_C( ambF ) );
  else
    return TC_RANGE_ERR;
};
