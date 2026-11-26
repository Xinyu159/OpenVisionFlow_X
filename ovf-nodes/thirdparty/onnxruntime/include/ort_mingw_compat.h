// MinGW 下 MSVC SAL 注解宏兜底（onnxruntime 头文件需要）
// 严格 ISO 模式下 MSVC 调用约定拼写未定义
#define _stdcall __stdcall
#define _cdecl __cdecl
#ifndef _Frees_ptr_opt_
#define _Frees_ptr_opt_
#define _In_
#define _In_opt_
#define _In_z_
#define _Inout_
#define _Out_
#define _Out_opt_
#define _Outptr_
#define _Outptr_opt_
#define _Ret_maybenull_
#define _Ret_notnull_
#define _Check_return_
#define _Post_invalid_
#define _Pre_notnull_
#define _Printf_format_string_
#define _Success_(x)
#define _Use_decl_annotations_
#define _When_(x,y)
#define _Acquires_lock_(x)
#define _Releases_lock_(x)
#define _Analysis_assume_(x)
#define _Const_
#define _Raises_SEH_exception_
#endif
