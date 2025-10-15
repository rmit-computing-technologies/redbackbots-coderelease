#pragma once

/**
 * @file asmjit_forwdec.hpp
 * 
 * Forward Declaration of asmjit Jit Runtime from CompiledNN library
 * This avoids including all of asmjit within everything that depends
 * on vision, and other ML tools.
*/

// Forward Declaration
namespace asmjit
{
  inline namespace _abi_1_9
  {
    class JitRuntime;
  }
}
