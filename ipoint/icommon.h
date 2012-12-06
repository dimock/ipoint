#pragma once

#undef USE_VERIFICATION

#ifdef USE_VERIFICATION
  #define THROW_IF(cond, msg) if ( cond ) throw std::runtime_error(msg); else;
  #define DO_VERIFY( action ) action;
  #define VERIFY_VECTOR(v) (_isnan((v).x) || _isnan((v).y) || _isnan((v).z))
#else
  #define THROW_IF(cond, msg) ;
  #define DO_VERIFY( action ) ;
#endif