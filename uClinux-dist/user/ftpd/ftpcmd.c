
/* A Bison parser, made by GNU Bison 2.4.1.  */

/* Skeleton implementation for Bison's Yacc-like parsers in C
   
      Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.
   
   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "2.4.1"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Copy the first part of user declarations.  */

/* Line 189 of yacc.c  */
#line 37 "ftpcmd.y"


#ifndef lint
static char sccsid[] = "@(#)ftpcmd.y	8.3 (Berkeley) 4/6/94";
#endif /* not lint */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <sys/types.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include <netinet/in.h>
#include <arpa/ftp.h>

#include <ctype.h>
#include <errno.h>
#include <pwd.h>
#include <setjmp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#ifdef TIME_WITH_SYS_TIME
# include <sys/time.h>
# include <time.h>
#else
# ifdef HAVE_SYS_TIME_H
#  include <sys/time.h>
# else
#  include <time.h>
# endif
#endif
#include <unistd.h>
#include <limits.h>
#ifdef HAVE_SYS_UTSNAME_H
#include <sys/utsname.h>
#endif
/* Include glob.h last, because it may define "const" which breaks
   system headers on some platforms. */
#include <glob.h>

#include "extern.h"

#if ! defined (NBBY) && defined (CHAR_BIT)
#define NBBY CHAR_BIT
#endif

off_t restart_point;

static char cbuf[512];           /* Command Buffer.  */
static char *fromname;
static int cmd_type;
static int cmd_form;
static int cmd_bytesz;

struct tab
{
  const char	*name;
  short	token;
  short	state;
  short	implemented;	/* 1 if command is implemented */
  const char	*help;
};

extern struct tab cmdtab[];
extern struct tab sitetab[];
static char *copy         __P ((char *));
static void help          __P ((struct tab *, char *));
static struct tab *lookup __P ((struct tab *, char *));
static void sizecmd       __P ((char *));
static int yylex          __P ((void));
static void yyerror       __P ((const char *s));



/* Line 189 of yacc.c  */
#line 154 "y.tab.c"

/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE 0
#endif


/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     A = 258,
     B = 259,
     C = 260,
     E = 261,
     F = 262,
     I = 263,
     L = 264,
     N = 265,
     P = 266,
     R = 267,
     S = 268,
     T = 269,
     SP = 270,
     CRLF = 271,
     COMMA = 272,
     USER = 273,
     PASS = 274,
     ACCT = 275,
     REIN = 276,
     QUIT = 277,
     PORT = 278,
     PASV = 279,
     TYPE = 280,
     STRU = 281,
     MODE = 282,
     RETR = 283,
     STOR = 284,
     APPE = 285,
     MLFL = 286,
     MAIL = 287,
     MSND = 288,
     MSOM = 289,
     MSAM = 290,
     MRSQ = 291,
     MRCP = 292,
     ALLO = 293,
     REST = 294,
     RNFR = 295,
     RNTO = 296,
     ABOR = 297,
     DELE = 298,
     CWD = 299,
     LIST = 300,
     NLST = 301,
     SITE = 302,
     STAT = 303,
     HELP = 304,
     NOOP = 305,
     MKD = 306,
     RMD = 307,
     PWD = 308,
     CDUP = 309,
     STOU = 310,
     SMNT = 311,
     SYST = 312,
     SIZE = 313,
     MDTM = 314,
     UMASK = 315,
     IDLE = 316,
     CHMOD = 317,
     LEXERR = 318,
     STRING = 319,
     NUMBER = 320
   };
#endif
/* Tokens.  */
#define A 258
#define B 259
#define C 260
#define E 261
#define F 262
#define I 263
#define L 264
#define N 265
#define P 266
#define R 267
#define S 268
#define T 269
#define SP 270
#define CRLF 271
#define COMMA 272
#define USER 273
#define PASS 274
#define ACCT 275
#define REIN 276
#define QUIT 277
#define PORT 278
#define PASV 279
#define TYPE 280
#define STRU 281
#define MODE 282
#define RETR 283
#define STOR 284
#define APPE 285
#define MLFL 286
#define MAIL 287
#define MSND 288
#define MSOM 289
#define MSAM 290
#define MRSQ 291
#define MRCP 292
#define ALLO 293
#define REST 294
#define RNFR 295
#define RNTO 296
#define ABOR 297
#define DELE 298
#define CWD 299
#define LIST 300
#define NLST 301
#define SITE 302
#define STAT 303
#define HELP 304
#define NOOP 305
#define MKD 306
#define RMD 307
#define PWD 308
#define CDUP 309
#define STOU 310
#define SMNT 311
#define SYST 312
#define SIZE 313
#define MDTM 314
#define UMASK 315
#define IDLE 316
#define CHMOD 317
#define LEXERR 318
#define STRING 319
#define NUMBER 320




#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
{

/* Line 214 of yacc.c  */
#line 117 "ftpcmd.y"

	int	i;
	char   *s;



/* Line 214 of yacc.c  */
#line 327 "y.tab.c"
} YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
#endif


/* Copy the second part of user declarations.  */


/* Line 264 of yacc.c  */
#line 339 "y.tab.c"

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static int
YYID (int yyi)
#else
static int
YYID (yyi)
    int yyi;
#endif
{
  return yyi;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef _STDLIB_H
#      define _STDLIB_H 1
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined _STDLIB_H \
       && ! ((defined YYMALLOC || defined malloc) \
	     && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef _STDLIB_H
#    define _STDLIB_H 1
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
	 || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  YYSIZE_T yyi;				\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (YYID (0))
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)				\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack_alloc, Stack, yysize);			\
	Stack = &yyptr->Stack_alloc;					\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  2
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   216

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  66
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  16
/* YYNRULES -- Number of rules.  */
#define YYNRULES  75
/* YYNRULES -- Number of states.  */
#define YYNSTATES  210

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   320

#define YYTRANSLATE(YYX)						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53,    54,
      55,    56,    57,    58,    59,    60,    61,    62,    63,    64,
      65
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const yytype_uint16 yyprhs[] =
{
       0,     0,     3,     4,     7,    10,    15,    20,    26,    30,
      35,    40,    45,    50,    59,    65,    71,    77,    81,    87,
      91,    97,   103,   106,   112,   118,   121,   125,   131,   134,
     139,   142,   148,   154,   158,   162,   167,   174,   180,   188,
     198,   203,   211,   217,   220,   226,   232,   235,   238,   244,
     249,   251,   252,   254,   256,   268,   270,   272,   274,   276,
     280,   282,   286,   288,   290,   294,   297,   299,   301,   303,
     305,   307,   309,   311,   313,   315
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int8 yyrhs[] =
{
      67,     0,    -1,    -1,    67,    68,    -1,    67,    69,    -1,
      18,    15,    70,    16,    -1,    19,    15,    71,    16,    -1,
      23,    81,    15,    73,    16,    -1,    24,    81,    16,    -1,
      25,    15,    75,    16,    -1,    26,    15,    76,    16,    -1,
      27,    15,    77,    16,    -1,    38,    15,    65,    16,    -1,
      38,    15,    65,    15,    12,    15,    65,    16,    -1,    28,
      81,    15,    78,    16,    -1,    29,    81,    15,    78,    16,
      -1,    30,    81,    15,    78,    16,    -1,    46,    81,    16,
      -1,    46,    81,    15,    64,    16,    -1,    45,    81,    16,
      -1,    45,    81,    15,    78,    16,    -1,    48,    81,    15,
      78,    16,    -1,    48,    16,    -1,    43,    81,    15,    78,
      16,    -1,    41,    81,    15,    78,    16,    -1,    42,    16,
      -1,    44,    81,    16,    -1,    44,    81,    15,    78,    16,
      -1,    49,    16,    -1,    49,    15,    64,    16,    -1,    50,
      16,    -1,    51,    81,    15,    78,    16,    -1,    52,    81,
      15,    78,    16,    -1,    53,    81,    16,    -1,    54,    81,
      16,    -1,    47,    15,    49,    16,    -1,    47,    15,    49,
      15,    64,    16,    -1,    47,    15,    60,    81,    16,    -1,
      47,    15,    60,    81,    15,    80,    16,    -1,    47,    15,
      62,    81,    15,    80,    15,    78,    16,    -1,    47,    15,
      61,    16,    -1,    47,    15,    81,    61,    15,    65,    16,
      -1,    55,    81,    15,    78,    16,    -1,    57,    16,    -1,
      58,    81,    15,    78,    16,    -1,    59,    81,    15,    78,
      16,    -1,    22,    16,    -1,     1,    16,    -1,    40,    81,
      15,    78,    16,    -1,    39,    15,    72,    16,    -1,    64,
      -1,    -1,    64,    -1,    65,    -1,    65,    17,    65,    17,
      65,    17,    65,    17,    65,    17,    65,    -1,    10,    -1,
      14,    -1,     5,    -1,     3,    -1,     3,    15,    74,    -1,
       6,    -1,     6,    15,    74,    -1,     8,    -1,     9,    -1,
       9,    15,    72,    -1,     9,    72,    -1,     7,    -1,    12,
      -1,    11,    -1,    13,    -1,     4,    -1,     5,    -1,    79,
      -1,    64,    -1,    65,    -1,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   151,   151,   153,   160,   164,   169,   175,   197,   202,
     237,   249,   261,   265,   269,   276,   283,   290,   295,   302,
     307,   314,   321,   325,   332,   345,   349,   354,   361,   365,
     382,   386,   393,   400,   405,   410,   414,   420,   430,   445,
     459,   465,   481,   488,   534,   551,   573,   578,   584,   595,
     610,   615,   618,   622,   626,   640,   644,   648,   655,   660,
     665,   670,   675,   679,   684,   690,   698,   702,   706,   713,
     717,   721,   728,   765,   769,   797
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "A", "B", "C", "E", "F", "I", "L", "N",
  "P", "R", "S", "T", "SP", "CRLF", "COMMA", "USER", "PASS", "ACCT",
  "REIN", "QUIT", "PORT", "PASV", "TYPE", "STRU", "MODE", "RETR", "STOR",
  "APPE", "MLFL", "MAIL", "MSND", "MSOM", "MSAM", "MRSQ", "MRCP", "ALLO",
  "REST", "RNFR", "RNTO", "ABOR", "DELE", "CWD", "LIST", "NLST", "SITE",
  "STAT", "HELP", "NOOP", "MKD", "RMD", "PWD", "CDUP", "STOU", "SMNT",
  "SYST", "SIZE", "MDTM", "UMASK", "IDLE", "CHMOD", "LEXERR", "STRING",
  "NUMBER", "$accept", "cmd_list", "cmd", "rcmd", "username", "password",
  "byte_size", "host_port", "form_code", "type_code", "struct_code",
  "mode_code", "pathname", "pathstring", "octal_number", "check_login", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298,   299,   300,   301,   302,   303,   304,
     305,   306,   307,   308,   309,   310,   311,   312,   313,   314,
     315,   316,   317,   318,   319,   320
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,    66,    67,    67,    67,    68,    68,    68,    68,    68,
      68,    68,    68,    68,    68,    68,    68,    68,    68,    68,
      68,    68,    68,    68,    68,    68,    68,    68,    68,    68,
      68,    68,    68,    68,    68,    68,    68,    68,    68,    68,
      68,    68,    68,    68,    68,    68,    68,    68,    69,    69,
      70,    71,    71,    72,    73,    74,    74,    74,    75,    75,
      75,    75,    75,    75,    75,    75,    76,    76,    76,    77,
      77,    77,    78,    79,    80,    81
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     0,     2,     2,     4,     4,     5,     3,     4,
       4,     4,     4,     8,     5,     5,     5,     3,     5,     3,
       5,     5,     2,     5,     5,     2,     3,     5,     2,     4,
       2,     5,     5,     3,     3,     4,     6,     5,     7,     9,
       4,     7,     5,     2,     5,     5,     2,     2,     5,     4,
       1,     0,     1,     1,    11,     1,     1,     1,     1,     3,
       1,     3,     1,     1,     3,     2,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     0
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       2,     0,     1,     0,     0,     0,     0,    75,    75,     0,
       0,     0,    75,    75,    75,     0,     0,    75,    75,     0,
      75,    75,    75,    75,     0,    75,     0,     0,    75,    75,
      75,    75,    75,     0,    75,    75,     3,     4,    47,     0,
      51,    46,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,    25,     0,     0,     0,     0,     0,
      22,     0,     0,    28,    30,     0,     0,     0,     0,     0,
      43,     0,     0,    50,     0,    52,     0,     0,     8,    58,
      60,    62,    63,     0,    66,    68,    67,     0,    70,    71,
      69,     0,     0,     0,     0,     0,    53,     0,     0,     0,
       0,     0,    26,     0,    19,     0,    17,     0,    75,     0,
      75,     0,     0,     0,     0,     0,    33,    34,     0,     0,
       0,     5,     6,     0,     0,     0,     0,     0,    65,     9,
      10,    11,    73,     0,    72,     0,     0,     0,    12,    49,
       0,     0,     0,     0,     0,     0,     0,    35,     0,    40,
       0,     0,     0,    29,     0,     0,     0,     0,     0,     0,
       7,    57,    55,    56,    59,    61,    64,    14,    15,    16,
       0,    48,    24,    23,    27,    20,    18,     0,     0,    37,
       0,     0,    21,    31,    32,    42,    44,    45,     0,     0,
      36,    74,     0,     0,     0,     0,     0,    38,     0,    41,
       0,    13,     0,     0,    39,     0,     0,     0,     0,    54
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     1,    36,    37,    74,    76,    97,   124,   164,    83,
      87,    91,   133,   134,   192,    42
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -94
static const yytype_int16 yypact[] =
{
     -94,    42,   -94,    -7,     8,    17,    15,   -94,   -94,    20,
      25,    44,   -94,   -94,   -94,    59,    62,   -94,   -94,    47,
     -94,   -94,   -94,   -94,    83,    88,    39,    98,   -94,   -94,
     -94,   -94,   -94,    99,   -94,   -94,   -94,   -94,   -94,    52,
      53,   -94,   103,   104,    70,     6,     7,   106,   107,   108,
      54,    60,   112,   113,   -94,   114,    41,    87,    91,   -46,
     -94,   115,    67,   -94,   -94,   117,   118,   119,   120,   122,
     -94,   123,   124,   -94,   125,   -94,   126,    69,   -94,   128,
     129,   -94,   -13,   130,   -94,   -94,   -94,   131,   -94,   -94,
     -94,   132,    76,    76,    76,    93,   -94,   133,    76,    76,
      76,    76,   -94,    76,   -94,    81,   -94,    95,   -94,   134,
     -94,    90,    76,   136,    76,    76,   -94,   -94,    76,    76,
      76,   -94,   -94,   137,   139,    48,    48,    60,   -94,   -94,
     -94,   -94,   -94,   140,   -94,   141,   142,   147,   -94,   -94,
     144,   145,   146,   148,   149,   150,    89,   -94,    97,   -94,
     152,   153,   154,   -94,   155,   156,   157,   158,   159,   111,
     -94,   -94,   -94,   -94,   -94,   -94,   -94,   -94,   -94,   -94,
     162,   -94,   -94,   -94,   -94,   -94,   -94,   163,   116,   -94,
     116,   121,   -94,   -94,   -94,   -94,   -94,   -94,   161,   127,
     -94,   -94,   164,   167,   168,   135,   169,   -94,    76,   -94,
     166,   -94,   171,   138,   -94,   172,   143,   173,   151,   -94
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int8 yypgoto[] =
{
     -94,   -94,   -94,   -94,   -94,   -94,   -78,   -94,    37,   -94,
     -94,   -94,   -93,   -94,   -11,    16
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -1
static const yytype_uint8 yytable[] =
{
     135,   136,   127,   107,   128,   140,   141,   142,   143,    38,
     144,    88,    89,    84,   108,   109,   110,    85,    86,   152,
      90,   154,   155,    39,    43,   156,   157,   158,    47,    48,
      49,    41,    40,    52,    53,    44,    55,    56,    57,    58,
      45,    61,     2,     3,    65,    66,    67,    68,    69,   166,
      71,    72,    96,   161,    62,    63,   101,   102,   162,    46,
       4,     5,   163,    54,     6,     7,     8,     9,    10,    11,
      12,    13,    14,    79,    50,   111,    80,    51,    81,    82,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    59,    33,
      34,    35,   103,   104,    60,   202,   105,   106,   137,   138,
     146,   147,   178,   179,    64,    70,    73,    75,    77,    95,
      78,    92,    93,    94,   148,    96,   150,    98,    99,   100,
     112,   113,   114,   115,   123,   116,   117,   118,   119,   120,
     132,   121,   122,   125,   126,   145,   129,   130,   131,   139,
     149,   151,   153,   177,   159,   160,   167,   168,   169,   170,
     171,   172,   173,   165,   174,   175,   176,   180,   181,   193,
     182,   183,   184,   185,   186,   187,   188,   189,   195,   190,
     197,   191,   198,   203,   199,   201,   194,   204,     0,   206,
     208,     0,   196,     0,     0,     0,     0,     0,     0,     0,
     200,     0,     0,   205,     0,     0,     0,     0,   207,     0,
       0,     0,     0,     0,     0,     0,   209
};

static const yytype_int16 yycheck[] =
{
      93,    94,    15,    49,    82,    98,    99,   100,   101,    16,
     103,     4,     5,     7,    60,    61,    62,    11,    12,   112,
      13,   114,   115,    15,     8,   118,   119,   120,    12,    13,
      14,    16,    15,    17,    18,    15,    20,    21,    22,    23,
      15,    25,     0,     1,    28,    29,    30,    31,    32,   127,
      34,    35,    65,     5,    15,    16,    15,    16,    10,    15,
      18,    19,    14,    16,    22,    23,    24,    25,    26,    27,
      28,    29,    30,     3,    15,    59,     6,    15,     8,     9,
      38,    39,    40,    41,    42,    43,    44,    45,    46,    47,
      48,    49,    50,    51,    52,    53,    54,    55,    15,    57,
      58,    59,    15,    16,    16,   198,    15,    16,    15,    16,
      15,    16,    15,    16,    16,    16,    64,    64,    15,    65,
      16,    15,    15,    15,   108,    65,   110,    15,    15,    15,
      15,    64,    15,    15,    65,    16,    16,    15,    15,    15,
      64,    16,    16,    15,    15,    64,    16,    16,    16,    16,
      16,    61,    16,    64,    17,    16,    16,    16,    16,    12,
      16,    16,    16,   126,    16,    16,    16,    15,    15,   180,
      16,    16,    16,    16,    16,    16,    65,    15,    17,    16,
      16,    65,    15,    17,    16,    16,    65,    16,    -1,    17,
      17,    -1,    65,    -1,    -1,    -1,    -1,    -1,    -1,    -1,
      65,    -1,    -1,    65,    -1,    -1,    -1,    -1,    65,    -1,
      -1,    -1,    -1,    -1,    -1,    -1,    65
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,    67,     0,     1,    18,    19,    22,    23,    24,    25,
      26,    27,    28,    29,    30,    38,    39,    40,    41,    42,
      43,    44,    45,    46,    47,    48,    49,    50,    51,    52,
      53,    54,    55,    57,    58,    59,    68,    69,    16,    15,
      15,    16,    81,    81,    15,    15,    15,    81,    81,    81,
      15,    15,    81,    81,    16,    81,    81,    81,    81,    15,
      16,    81,    15,    16,    16,    81,    81,    81,    81,    81,
      16,    81,    81,    64,    70,    64,    71,    15,    16,     3,
       6,     8,     9,    75,     7,    11,    12,    76,     4,     5,
      13,    77,    15,    15,    15,    65,    65,    72,    15,    15,
      15,    15,    16,    15,    16,    15,    16,    49,    60,    61,
      62,    81,    15,    64,    15,    15,    16,    16,    15,    15,
      15,    16,    16,    65,    73,    15,    15,    15,    72,    16,
      16,    16,    64,    78,    79,    78,    78,    15,    16,    16,
      78,    78,    78,    78,    78,    64,    15,    16,    81,    16,
      81,    61,    78,    16,    78,    78,    78,    78,    78,    17,
      16,     5,    10,    14,    74,    74,    72,    16,    16,    16,
      12,    16,    16,    16,    16,    16,    16,    64,    15,    16,
      15,    15,    16,    16,    16,    16,    16,    16,    65,    15,
      16,    65,    80,    80,    65,    17,    65,    16,    15,    16,
      65,    16,    78,    17,    16,    65,    17,    65,    17,    65
};

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */

#define YYFAIL		goto yyerrlab

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      yytoken = YYTRANSLATE (yychar);				\
      YYPOPSTACK (1);						\
      goto yybackup;						\
    }								\
  else								\
    {								\
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;							\
    }								\
while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
      if (YYID (N))                                                    \
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	}								\
    while (YYID (0))
#endif


/* YY_LOCATION_PRINT -- Print the location on the stream.
   This macro was not mandated originally: define only if we know
   we won't break user code: when these are the locations we know.  */

#ifndef YY_LOCATION_PRINT
# if YYLTYPE_IS_TRIVIAL
#  define YY_LOCATION_PRINT(File, Loc)			\
     fprintf (File, "%d.%d-%d.%d",			\
	      (Loc).first_line, (Loc).first_column,	\
	      (Loc).last_line,  (Loc).last_column)
# else
#  define YY_LOCATION_PRINT(File, Loc) ((void) 0)
# endif
#endif


/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (YYLEX_PARAM)
#else
# define YYLEX yylex ()
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
do {									  \
  if (yydebug)								  \
    {									  \
      YYFPRINTF (stderr, "%s ", Title);					  \
      yy_symbol_print (stderr,						  \
		  Type, Value); \
      YYFPRINTF (stderr, "\n");						  \
    }									  \
} while (YYID (0))


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_value_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# else
  YYUSE (yyoutput);
# endif
  switch (yytype)
    {
      default:
	break;
    }
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (yytype < YYNTOKENS)
    YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_stack_print (yytype_int16 *yybottom, yytype_int16 *yytop)
#else
static void
yy_stack_print (yybottom, yytop)
    yytype_int16 *yybottom;
    yytype_int16 *yytop;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_reduce_print (YYSTYPE *yyvsp, int yyrule)
#else
static void
yy_reduce_print (yyvsp, yyrule)
    YYSTYPE *yyvsp;
    int yyrule;
#endif
{
  int yynrhs = yyr2[yyrule];
  int yyi;
  unsigned long int yylno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
	     yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
		       &(yyvsp[(yyi + 1) - (yynrhs)])
		       		       );
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (yyvsp, Rule); \
} while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static YYSIZE_T
yystrlen (const char *yystr)
#else
static YYSIZE_T
yystrlen (yystr)
    const char *yystr;
#endif
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static char *
yystpcpy (char *yydest, const char *yysrc)
#else
static char *
yystpcpy (yydest, yysrc)
    char *yydest;
    const char *yysrc;
#endif
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
	switch (*++yyp)
	  {
	  case '\'':
	  case ',':
	    goto do_not_strip_quotes;

	  case '\\':
	    if (*++yyp != '\\')
	      goto do_not_strip_quotes;
	    /* Fall through.  */
	  default:
	    if (yyres)
	      yyres[yyn] = *yyp;
	    yyn++;
	    break;

	  case '"':
	    if (yyres)
	      yyres[yyn] = '\0';
	    return yyn;
	  }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into YYRESULT an error message about the unexpected token
   YYCHAR while in state YYSTATE.  Return the number of bytes copied,
   including the terminating null byte.  If YYRESULT is null, do not
   copy anything; just return the number of bytes that would be
   copied.  As a special case, return 0 if an ordinary "syntax error"
   message will do.  Return YYSIZE_MAXIMUM if overflow occurs during
   size calculation.  */
static YYSIZE_T
yysyntax_error (char *yyresult, int yystate, int yychar)
{
  int yyn = yypact[yystate];

  if (! (YYPACT_NINF < yyn && yyn <= YYLAST))
    return 0;
  else
    {
      int yytype = YYTRANSLATE (yychar);
      YYSIZE_T yysize0 = yytnamerr (0, yytname[yytype]);
      YYSIZE_T yysize = yysize0;
      YYSIZE_T yysize1;
      int yysize_overflow = 0;
      enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
      char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
      int yyx;

# if 0
      /* This is so xgettext sees the translatable formats that are
	 constructed on the fly.  */
      YY_("syntax error, unexpected %s");
      YY_("syntax error, unexpected %s, expecting %s");
      YY_("syntax error, unexpected %s, expecting %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s");
# endif
      char *yyfmt;
      char const *yyf;
      static char const yyunexpected[] = "syntax error, unexpected %s";
      static char const yyexpecting[] = ", expecting %s";
      static char const yyor[] = " or %s";
      char yyformat[sizeof yyunexpected
		    + sizeof yyexpecting - 1
		    + ((YYERROR_VERBOSE_ARGS_MAXIMUM - 2)
		       * (sizeof yyor - 1))];
      char const *yyprefix = yyexpecting;

      /* Start YYX at -YYN if negative to avoid negative indexes in
	 YYCHECK.  */
      int yyxbegin = yyn < 0 ? -yyn : 0;

      /* Stay within bounds of both yycheck and yytname.  */
      int yychecklim = YYLAST - yyn + 1;
      int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
      int yycount = 1;

      yyarg[0] = yytname[yytype];
      yyfmt = yystpcpy (yyformat, yyunexpected);

      for (yyx = yyxbegin; yyx < yyxend; ++yyx)
	if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
	  {
	    if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
	      {
		yycount = 1;
		yysize = yysize0;
		yyformat[sizeof yyunexpected - 1] = '\0';
		break;
	      }
	    yyarg[yycount++] = yytname[yyx];
	    yysize1 = yysize + yytnamerr (0, yytname[yyx]);
	    yysize_overflow |= (yysize1 < yysize);
	    yysize = yysize1;
	    yyfmt = yystpcpy (yyfmt, yyprefix);
	    yyprefix = yyor;
	  }

      yyf = YY_(yyformat);
      yysize1 = yysize + yystrlen (yyf);
      yysize_overflow |= (yysize1 < yysize);
      yysize = yysize1;

      if (yysize_overflow)
	return YYSIZE_MAXIMUM;

      if (yyresult)
	{
	  /* Avoid sprintf, as that infringes on the user's name space.
	     Don't have undefined behavior even if the translation
	     produced a string with the wrong number of "%s"s.  */
	  char *yyp = yyresult;
	  int yyi = 0;
	  while ((*yyp = *yyf) != '\0')
	    {
	      if (*yyp == '%' && yyf[1] == 's' && yyi < yycount)
		{
		  yyp += yytnamerr (yyp, yyarg[yyi++]);
		  yyf += 2;
		}
	      else
		{
		  yyp++;
		  yyf++;
		}
	    }
	}
      return yysize;
    }
}
#endif /* YYERROR_VERBOSE */


/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yymsg, yytype, yyvaluep)
    const char *yymsg;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  YYUSE (yyvaluep);

  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  switch (yytype)
    {

      default:
	break;
    }
}

/* Prevent warnings from -Wmissing-prototypes.  */
#ifdef YYPARSE_PARAM
#if defined __STDC__ || defined __cplusplus
int yyparse (void *YYPARSE_PARAM);
#else
int yyparse ();
#endif
#else /* ! YYPARSE_PARAM */
#if defined __STDC__ || defined __cplusplus
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */


/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*-------------------------.
| yyparse or yypush_parse.  |
`-------------------------*/

#ifdef YYPARSE_PARAM
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void *YYPARSE_PARAM)
#else
int
yyparse (YYPARSE_PARAM)
    void *YYPARSE_PARAM;
#endif
#else /* ! YYPARSE_PARAM */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{


    int yystate;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       `yyss': related to states.
       `yyvs': related to semantic values.

       Refer to the stacks thru separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;

    YYSIZE_T yystacksize;

  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  yytoken = 0;
  yyss = yyssa;
  yyvs = yyvsa;
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */
  yyssp = yyss;
  yyvsp = yyvs;

  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack.  Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	yytype_int16 *yyss1 = yyss;

	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow (YY_("memory exhausted"),
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),
		    &yystacksize);

	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	yytype_int16 *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyexhaustedlab;
	YYSTACK_RELOCATE (yyss_alloc, yyss);
	YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yyn == YYPACT_NINF)
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yyn == 0 || yyn == YYTABLE_NINF)
	goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
  yychar = YYEMPTY;

  yystate = yyn;
  *++yyvsp = yylval;

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 3:

/* Line 1455 of yacc.c  */
#line 154 "ftpcmd.y"
    {
			if (fromname != NULL)
				free (fromname);
			fromname = (char *) 0;
			restart_point = (off_t) 0;
		}
    break;

  case 5:

/* Line 1455 of yacc.c  */
#line 165 "ftpcmd.y"
    {
			user((yyvsp[(3) - (4)].s));
			free((yyvsp[(3) - (4)].s));
		}
    break;

  case 6:

/* Line 1455 of yacc.c  */
#line 170 "ftpcmd.y"
    {
			pass((yyvsp[(3) - (4)].s));
			memset ((yyvsp[(3) - (4)].s), 0, strlen ((yyvsp[(3) - (4)].s)));
			free((yyvsp[(3) - (4)].s));
		}
    break;

  case 7:

/* Line 1455 of yacc.c  */
#line 176 "ftpcmd.y"
    {
			usedefault = 0;
			if (pdata >= 0) {
				(void) close(pdata);
				pdata = -1;
			}
			if ((yyvsp[(2) - (5)].i)) {
				if (memcmp (&his_addr.sin_addr,
					&data_dest.sin_addr,
					sizeof (data_dest.sin_addr)) == 0 &&
					ntohs (data_dest.sin_port) >
					IPPORT_RESERVED) {
					reply (200, "PORT command sucessful.");
				}
				else {
					memset (&data_dest, 0,
						sizeof (data_dest));
					reply(500, "Illegal PORT Command");
				}
			}
		}
    break;

  case 8:

/* Line 1455 of yacc.c  */
#line 198 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (3)].i))
				passive();
		}
    break;

  case 9:

/* Line 1455 of yacc.c  */
#line 203 "ftpcmd.y"
    {
			switch (cmd_type) {

			case TYPE_A:
				if (cmd_form == FORM_N) {
					reply(200, "Type set to A.");
					type = cmd_type;
					form = cmd_form;
				} else
					reply(504, "Form must be N.");
				break;

			case TYPE_E:
				reply(504, "Type E not implemented.");
				break;

			case TYPE_I:
				reply(200, "Type set to I.");
				type = cmd_type;
				break;

			case TYPE_L:
#if defined (NBBY) && NBBY == 8
				if (cmd_bytesz == 8) {
					reply(200,
					    "Type set to L (byte size 8).");
					type = cmd_type;
				} else
					reply(504, "Byte size must be 8.");
#else /* NBBY == 8 */
				UNIMPLEMENTED for NBBY != 8
#endif /* NBBY == 8 */
			}
		}
    break;

  case 10:

/* Line 1455 of yacc.c  */
#line 238 "ftpcmd.y"
    {
			switch ((yyvsp[(3) - (4)].i)) {

			case STRU_F:
				reply(200, "STRU F ok.");
				break;

			default:
				reply(504, "Unimplemented STRU type.");
			}
		}
    break;

  case 11:

/* Line 1455 of yacc.c  */
#line 250 "ftpcmd.y"
    {
			switch ((yyvsp[(3) - (4)].i)) {

			case MODE_S:
				reply(200, "MODE S ok.");
				break;

			default:
				reply(502, "Unimplemented MODE type.");
			}
		}
    break;

  case 12:

/* Line 1455 of yacc.c  */
#line 262 "ftpcmd.y"
    {
			reply(202, "ALLO command ignored.");
		}
    break;

  case 13:

/* Line 1455 of yacc.c  */
#line 266 "ftpcmd.y"
    {
			reply(202, "ALLO command ignored.");
		}
    break;

  case 14:

/* Line 1455 of yacc.c  */
#line 270 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				retrieve((char *) 0, (yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 15:

/* Line 1455 of yacc.c  */
#line 277 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				store((yyvsp[(4) - (5)].s), "w", 0);
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 16:

/* Line 1455 of yacc.c  */
#line 284 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				store((yyvsp[(4) - (5)].s), "a", 0);
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 17:

/* Line 1455 of yacc.c  */
#line 291 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (3)].i))
				send_file_list(".");
		}
    break;

  case 18:

/* Line 1455 of yacc.c  */
#line 296 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				send_file_list((yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 19:

/* Line 1455 of yacc.c  */
#line 303 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (3)].i))
				retrieve("/bin/ls -lgA", "");
		}
    break;

  case 20:

/* Line 1455 of yacc.c  */
#line 308 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				retrieve("/bin/ls -lgA %s", (yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 21:

/* Line 1455 of yacc.c  */
#line 315 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				statfilecmd((yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 22:

/* Line 1455 of yacc.c  */
#line 322 "ftpcmd.y"
    {
			statcmd();
		}
    break;

  case 23:

/* Line 1455 of yacc.c  */
#line 326 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				delete((yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 24:

/* Line 1455 of yacc.c  */
#line 333 "ftpcmd.y"
    {
		    if ((yyvsp[(2) - (5)].i)) {
			if (fromname) {
				renamecmd(fromname, (yyvsp[(4) - (5)].s));
				free(fromname);
				fromname = (char *) 0;
			} else {
				reply(503, "Bad sequence of commands.");
			}
		    }
		    free ((yyvsp[(4) - (5)].s));
		}
    break;

  case 25:

/* Line 1455 of yacc.c  */
#line 346 "ftpcmd.y"
    {
			reply(225, "ABOR command successful.");
		}
    break;

  case 26:

/* Line 1455 of yacc.c  */
#line 350 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (3)].i))
				cwd(cred.homedir);
		}
    break;

  case 27:

/* Line 1455 of yacc.c  */
#line 355 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				cwd((yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 28:

/* Line 1455 of yacc.c  */
#line 362 "ftpcmd.y"
    {
			help(cmdtab, (char *) 0);
		}
    break;

  case 29:

/* Line 1455 of yacc.c  */
#line 366 "ftpcmd.y"
    {
			char *cp = (yyvsp[(3) - (4)].s);

			if (strncasecmp(cp, "SITE", 4) == 0) {
				cp = (yyvsp[(3) - (4)].s) + 4;
				if (*cp == ' ')
					cp++;
				if (*cp)
					help(sitetab, cp);
				else
					help(sitetab, (char *) 0);
			} else
				help(cmdtab, (yyvsp[(3) - (4)].s));
			if ((yyvsp[(3) - (4)].s) != NULL)
			    free ((yyvsp[(3) - (4)].s));
		}
    break;

  case 30:

/* Line 1455 of yacc.c  */
#line 383 "ftpcmd.y"
    {
			reply(200, "NOOP command successful.");
		}
    break;

  case 31:

/* Line 1455 of yacc.c  */
#line 387 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				makedir((yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 32:

/* Line 1455 of yacc.c  */
#line 394 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				removedir((yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 33:

/* Line 1455 of yacc.c  */
#line 401 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (3)].i))
				pwd();
		}
    break;

  case 34:

/* Line 1455 of yacc.c  */
#line 406 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (3)].i))
				cwd("..");
		}
    break;

  case 35:

/* Line 1455 of yacc.c  */
#line 411 "ftpcmd.y"
    {
			help(sitetab, (char *) 0);
		}
    break;

  case 36:

/* Line 1455 of yacc.c  */
#line 415 "ftpcmd.y"
    {
			help(sitetab, (yyvsp[(5) - (6)].s));
			if ((yyvsp[(5) - (6)].s) != NULL)
			    free ((yyvsp[(5) - (6)].s));
		}
    break;

  case 37:

/* Line 1455 of yacc.c  */
#line 421 "ftpcmd.y"
    {
			int oldmask;

			if ((yyvsp[(4) - (5)].i)) {
				oldmask = umask(0);
				(void) umask(oldmask);
				reply(200, "Current UMASK is %03o", oldmask);
			}
		}
    break;

  case 38:

/* Line 1455 of yacc.c  */
#line 431 "ftpcmd.y"
    {
			int oldmask;

			if ((yyvsp[(4) - (7)].i)) {
				if (((yyvsp[(6) - (7)].i) == -1) || ((yyvsp[(6) - (7)].i) > 0777)) {
					reply(501, "Bad UMASK value");
				} else {
					oldmask = umask((yyvsp[(6) - (7)].i));
					reply(200,
					    "UMASK set to %03o (was %03o)",
					    (yyvsp[(6) - (7)].i), oldmask);
				}
			}
		}
    break;

  case 39:

/* Line 1455 of yacc.c  */
#line 446 "ftpcmd.y"
    {
			if ((yyvsp[(4) - (9)].i) && ((yyvsp[(8) - (9)].s) != NULL)) {
				if ((yyvsp[(6) - (9)].i) > 0777)
					reply(501,
				"CHMOD: Mode value must be between 0 and 0777");
				else if (chmod((yyvsp[(8) - (9)].s), (yyvsp[(6) - (9)].i)) < 0)
					perror_reply(550, (yyvsp[(8) - (9)].s));
				else
					reply(200, "CHMOD command successful.");
			}
			if ((yyvsp[(8) - (9)].s) != NULL)
				free((yyvsp[(8) - (9)].s));
		}
    break;

  case 40:

/* Line 1455 of yacc.c  */
#line 460 "ftpcmd.y"
    {
			reply(200,
			    "Current IDLE time limit is %d seconds; max %d",
				timeout, maxtimeout);
		}
    break;

  case 41:

/* Line 1455 of yacc.c  */
#line 466 "ftpcmd.y"
    {
		    	if ((yyvsp[(3) - (7)].i)) {
			    if ((yyvsp[(6) - (7)].i) < 30 || (yyvsp[(6) - (7)].i) > maxtimeout) {
				reply (501,
			"Maximum IDLE time must be between 30 and %d seconds",
					maxtimeout);
			    } else {
				timeout = (yyvsp[(6) - (7)].i);
				(void) alarm((unsigned) timeout);
				reply(200,
					"Maximum IDLE time set to %d seconds",
					timeout);
			    }
			}
		}
    break;

  case 42:

/* Line 1455 of yacc.c  */
#line 482 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				store((yyvsp[(4) - (5)].s), "w", 1);
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 43:

/* Line 1455 of yacc.c  */
#line 489 "ftpcmd.y"
    {
		        const char *sys_type; /* Official rfc-defined os type.  */
			char *version = 0; /* A more specific type. */

#ifdef HAVE_UNAME
			struct utsname u;
			if (uname (&u) == 0) {
				version =
				  malloc (strlen (u.sysname)
					  + 1 + strlen (u.release) + 1);
				if (version)
					sprintf (version, "%s %s",
						 u.sysname, u.release);
		        }
#else
#ifdef BSD
			version = "BSD";
#endif
#endif

#ifdef unix
			sys_type = "UNIX";
#else
			sys_type = "UNKNOWN";
#endif

			if (version)
				reply(215, "%s Type: L%d Version: %s",
				      sys_type, NBBY, version);
			else
				reply(215, "%s Type: L%d", sys_type, NBBY);

#ifdef HAVE_UNAME
			if (version)
				free (version);
#endif
		}
    break;

  case 44:

/* Line 1455 of yacc.c  */
#line 535 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL)
				sizecmd((yyvsp[(4) - (5)].s));
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 45:

/* Line 1455 of yacc.c  */
#line 552 "ftpcmd.y"
    {
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s) != NULL) {
				struct stat stbuf;
				if (stat((yyvsp[(4) - (5)].s), &stbuf) < 0)
					reply(550, "%s: %s",
					    (yyvsp[(4) - (5)].s), strerror(errno));
				else if (!S_ISREG(stbuf.st_mode)) {
					reply(550, "%s: not a plain file.", (yyvsp[(4) - (5)].s));
				} else {
					struct tm *t;
					t = gmtime(&stbuf.st_mtime);
					reply(213,
					    "%04d%02d%02d%02d%02d%02d",
					    1900 + t->tm_year, t->tm_mon+1,
					    t->tm_mday, t->tm_hour, t->tm_min,
					    t->tm_sec);
				}
			}
			if ((yyvsp[(4) - (5)].s) != NULL)
				free((yyvsp[(4) - (5)].s));
		}
    break;

  case 46:

/* Line 1455 of yacc.c  */
#line 574 "ftpcmd.y"
    {
			reply(221, "Goodbye.");
			dologout(0);
		}
    break;

  case 47:

/* Line 1455 of yacc.c  */
#line 579 "ftpcmd.y"
    {
			yyerrok;
		}
    break;

  case 48:

/* Line 1455 of yacc.c  */
#line 585 "ftpcmd.y"
    {
			restart_point = (off_t) 0;
			if ((yyvsp[(2) - (5)].i) && (yyvsp[(4) - (5)].s)) {
			    if (fromname != NULL)
				free (fromname);
			    fromname = renamefrom((yyvsp[(4) - (5)].s));
			}
			if (fromname == (char *) 0 && (yyvsp[(4) - (5)].s))
			    free((yyvsp[(4) - (5)].s));
		}
    break;

  case 49:

/* Line 1455 of yacc.c  */
#line 596 "ftpcmd.y"
    {
		    	if (fromname != NULL)
				free (fromname);
			fromname = (char *) 0;
			restart_point = (yyvsp[(3) - (4)].i);	/* XXX $3 is only "int" */
			reply(350,
			      (sizeof(restart_point) > sizeof(long)
			       ? "Restarting at %qd. %s"
			       : "Restarting at %ld. %s"), restart_point,
			    "Send STORE or RETRIEVE to initiate transfer.");
		}
    break;

  case 51:

/* Line 1455 of yacc.c  */
#line 615 "ftpcmd.y"
    {
			(yyval.s) = (char *)calloc(1, sizeof(char));
		}
    break;

  case 54:

/* Line 1455 of yacc.c  */
#line 628 "ftpcmd.y"
    {
			char *a, *p;

			a = (char *)&data_dest.sin_addr;
			a[0] = (yyvsp[(1) - (11)].i); a[1] = (yyvsp[(3) - (11)].i); a[2] = (yyvsp[(5) - (11)].i); a[3] = (yyvsp[(7) - (11)].i);
			p = (char *)&data_dest.sin_port;
			p[0] = (yyvsp[(9) - (11)].i); p[1] = (yyvsp[(11) - (11)].i);
			data_dest.sin_family = AF_INET;
		}
    break;

  case 55:

/* Line 1455 of yacc.c  */
#line 641 "ftpcmd.y"
    {
			(yyval.i) = FORM_N;
		}
    break;

  case 56:

/* Line 1455 of yacc.c  */
#line 645 "ftpcmd.y"
    {
			(yyval.i) = FORM_T;
		}
    break;

  case 57:

/* Line 1455 of yacc.c  */
#line 649 "ftpcmd.y"
    {
			(yyval.i) = FORM_C;
		}
    break;

  case 58:

/* Line 1455 of yacc.c  */
#line 656 "ftpcmd.y"
    {
			cmd_type = TYPE_A;
			cmd_form = FORM_N;
		}
    break;

  case 59:

/* Line 1455 of yacc.c  */
#line 661 "ftpcmd.y"
    {
			cmd_type = TYPE_A;
			cmd_form = (yyvsp[(3) - (3)].i);
		}
    break;

  case 60:

/* Line 1455 of yacc.c  */
#line 666 "ftpcmd.y"
    {
			cmd_type = TYPE_E;
			cmd_form = FORM_N;
		}
    break;

  case 61:

/* Line 1455 of yacc.c  */
#line 671 "ftpcmd.y"
    {
			cmd_type = TYPE_E;
			cmd_form = (yyvsp[(3) - (3)].i);
		}
    break;

  case 62:

/* Line 1455 of yacc.c  */
#line 676 "ftpcmd.y"
    {
			cmd_type = TYPE_I;
		}
    break;

  case 63:

/* Line 1455 of yacc.c  */
#line 680 "ftpcmd.y"
    {
			cmd_type = TYPE_L;
			cmd_bytesz = NBBY;
		}
    break;

  case 64:

/* Line 1455 of yacc.c  */
#line 685 "ftpcmd.y"
    {
			cmd_type = TYPE_L;
			cmd_bytesz = (yyvsp[(3) - (3)].i);
		}
    break;

  case 65:

/* Line 1455 of yacc.c  */
#line 691 "ftpcmd.y"
    {
			cmd_type = TYPE_L;
			cmd_bytesz = (yyvsp[(2) - (2)].i);
		}
    break;

  case 66:

/* Line 1455 of yacc.c  */
#line 699 "ftpcmd.y"
    {
			(yyval.i) = STRU_F;
		}
    break;

  case 67:

/* Line 1455 of yacc.c  */
#line 703 "ftpcmd.y"
    {
			(yyval.i) = STRU_R;
		}
    break;

  case 68:

/* Line 1455 of yacc.c  */
#line 707 "ftpcmd.y"
    {
			(yyval.i) = STRU_P;
		}
    break;

  case 69:

/* Line 1455 of yacc.c  */
#line 714 "ftpcmd.y"
    {
			(yyval.i) = MODE_S;
		}
    break;

  case 70:

/* Line 1455 of yacc.c  */
#line 718 "ftpcmd.y"
    {
			(yyval.i) = MODE_B;
		}
    break;

  case 71:

/* Line 1455 of yacc.c  */
#line 722 "ftpcmd.y"
    {
			(yyval.i) = MODE_C;
		}
    break;

  case 72:

/* Line 1455 of yacc.c  */
#line 729 "ftpcmd.y"
    {
			/*
			 * Problem: this production is used for all pathname
			 * processing, but only gives a 550 error reply.
			 * This is a valid reply in some cases but not in others.
			 */
			if (cred.logged_in && (yyvsp[(1) - (1)].s) && *(yyvsp[(1) - (1)].s) == '~') {
				glob_t gl;
				int flags = GLOB_NOCHECK;

#ifdef GLOB_BRACE
				flags |= GLOB_BRACE;
#endif
#ifdef GLOB_QUOTE
				flags |= GLOB_QUOTE;
#endif
#ifdef GLOB_TILDE
				flags |= GLOB_TILDE;
#endif

				memset(&gl, 0, sizeof(gl));
				if (glob((yyvsp[(1) - (1)].s), flags, NULL, &gl) ||
				    gl.gl_pathc == 0) {
					reply(550, "not found");
					(yyval.s) = NULL;
				} else {
					(yyval.s) = strdup(gl.gl_pathv[0]);
				}
				globfree(&gl);
				free((yyvsp[(1) - (1)].s));
			} else
				(yyval.s) = (yyvsp[(1) - (1)].s);
		}
    break;

  case 74:

/* Line 1455 of yacc.c  */
#line 770 "ftpcmd.y"
    {
			int ret, dec, multby, digit;

			/*
			 * Convert a number that was read as decimal number
			 * to what it would be if it had been read as octal.
			 */
			dec = (yyvsp[(1) - (1)].i);
			multby = 1;
			ret = 0;
			while (dec) {
				digit = dec%10;
				if (digit > 7) {
					ret = -1;
					break;
				}
				ret += digit * multby;
				multby *= 8;
				dec /= 10;
			}
			(yyval.i) = ret;
		}
    break;

  case 75:

/* Line 1455 of yacc.c  */
#line 797 "ftpcmd.y"
    {
			if (cred.logged_in)
				(yyval.i) = 1;
			else {
				reply(530, "Please login with USER and PASS.");
				(yyval.i) = 0;
			}
		}
    break;



/* Line 1455 of yacc.c  */
#line 2625 "y.tab.c"
      default: break;
    }
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;

  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
      {
	YYSIZE_T yysize = yysyntax_error (0, yystate, yychar);
	if (yymsg_alloc < yysize && yymsg_alloc < YYSTACK_ALLOC_MAXIMUM)
	  {
	    YYSIZE_T yyalloc = 2 * yysize;
	    if (! (yysize <= yyalloc && yyalloc <= YYSTACK_ALLOC_MAXIMUM))
	      yyalloc = YYSTACK_ALLOC_MAXIMUM;
	    if (yymsg != yymsgbuf)
	      YYSTACK_FREE (yymsg);
	    yymsg = (char *) YYSTACK_ALLOC (yyalloc);
	    if (yymsg)
	      yymsg_alloc = yyalloc;
	    else
	      {
		yymsg = yymsgbuf;
		yymsg_alloc = sizeof yymsgbuf;
	      }
	  }

	if (0 < yysize && yysize <= yymsg_alloc)
	  {
	    (void) yysyntax_error (yymsg, yystate, yychar);
	    yyerror (yymsg);
	  }
	else
	  {
	    yyerror (YY_("syntax error"));
	    if (yysize != 0)
	      goto yyexhaustedlab;
	  }
      }
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
	 error, discard it.  */

      if (yychar <= YYEOF)
	{
	  /* Return failure if at end of input.  */
	  if (yychar == YYEOF)
	    YYABORT;
	}
      else
	{
	  yydestruct ("Error: discarding",
		      yytoken, &yylval);
	  yychar = YYEMPTY;
	}
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule which action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;	/* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (yyn != YYPACT_NINF)
	{
	  yyn += YYTERROR;
	  if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
	    {
	      yyn = yytable[yyn];
	      if (0 < yyn)
		break;
	    }
	}

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
	YYABORT;


      yydestruct ("Error: popping",
		  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  *++yyvsp = yylval;


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#if !defined(yyoverflow) || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
     yydestruct ("Cleanup: discarding lookahead",
		 yytoken, &yylval);
  /* Do not reclaim the symbols of the rule which action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
		  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  /* Make sure YYID is used.  */
  return YYID (yyresult);
}



/* Line 1675 of yacc.c  */
#line 807 "ftpcmd.y"


#define	CMD	0	/* beginning of command */
#define	ARGS	1	/* expect miscellaneous arguments */
#define	STR1	2	/* expect SP followed by STRING */
#define	STR2	3	/* expect STRING */
#define	OSTR	4	/* optional SP then STRING */
#define	ZSTR1	5	/* SP then optional STRING */
#define	ZSTR2	6	/* optional STRING after SP */
#define	SITECMD	7	/* SITE command */
#define	NSTR	8	/* Number followed by a string */

struct tab cmdtab[] = {		/* In order defined in RFC 765 */
	{ "USER", USER, STR1, 1,	"<sp> username" },
	{ "PASS", PASS, ZSTR1, 1,	"<sp> password" },
	{ "ACCT", ACCT, STR1, 0,	"(specify account)" },
	{ "SMNT", SMNT, ARGS, 0,	"(structure mount)" },
	{ "REIN", REIN, ARGS, 0,	"(reinitialize server state)" },
	{ "QUIT", QUIT, ARGS, 1,	"(terminate service)", },
	{ "PORT", PORT, ARGS, 1,	"<sp> b0, b1, b2, b3, b4" },
	{ "PASV", PASV, ARGS, 1,	"(set server in passive mode)" },
	{ "TYPE", TYPE, ARGS, 1,	"<sp> [ A | E | I | L ]" },
	{ "STRU", STRU, ARGS, 1,	"(specify file structure)" },
	{ "MODE", MODE, ARGS, 1,	"(specify transfer mode)" },
	{ "RETR", RETR, STR1, 1,	"<sp> file-name" },
	{ "STOR", STOR, STR1, 1,	"<sp> file-name" },
	{ "APPE", APPE, STR1, 1,	"<sp> file-name" },
	{ "MLFL", MLFL, OSTR, 0,	"(mail file)" },
	{ "MAIL", MAIL, OSTR, 0,	"(mail to user)" },
	{ "MSND", MSND, OSTR, 0,	"(mail send to terminal)" },
	{ "MSOM", MSOM, OSTR, 0,	"(mail send to terminal or mailbox)" },
	{ "MSAM", MSAM, OSTR, 0,	"(mail send to terminal and mailbox)" },
	{ "MRSQ", MRSQ, OSTR, 0,	"(mail recipient scheme question)" },
	{ "MRCP", MRCP, STR1, 0,	"(mail recipient)" },
	{ "ALLO", ALLO, ARGS, 1,	"allocate storage (vacuously)" },
	{ "REST", REST, ARGS, 1,	"<sp> offset (restart command)" },
	{ "RNFR", RNFR, STR1, 1,	"<sp> file-name" },
	{ "RNTO", RNTO, STR1, 1,	"<sp> file-name" },
	{ "ABOR", ABOR, ARGS, 1,	"(abort operation)" },
	{ "DELE", DELE, STR1, 1,	"<sp> file-name" },
	{ "CWD",  CWD,  OSTR, 1,	"[ <sp> directory-name ]" },
	{ "XCWD", CWD,	OSTR, 1,	"[ <sp> directory-name ]" },
	{ "LIST", LIST, OSTR, 1,	"[ <sp> path-name ]" },
	{ "NLST", NLST, OSTR, 1,	"[ <sp> path-name ]" },
	{ "SITE", SITE, SITECMD, 1,	"site-cmd [ <sp> arguments ]" },
	{ "SYST", SYST, ARGS, 1,	"(get type of operating system)" },
	{ "STAT", STAT, OSTR, 1,	"[ <sp> path-name ]" },
	{ "HELP", HELP, OSTR, 1,	"[ <sp> <string> ]" },
	{ "NOOP", NOOP, ARGS, 1,	"" },
	{ "MKD",  MKD,  STR1, 1,	"<sp> path-name" },
	{ "XMKD", MKD,  STR1, 1,	"<sp> path-name" },
	{ "RMD",  RMD,  STR1, 1,	"<sp> path-name" },
	{ "XRMD", RMD,  STR1, 1,	"<sp> path-name" },
	{ "PWD",  PWD,  ARGS, 1,	"(return current directory)" },
	{ "XPWD", PWD,  ARGS, 1,	"(return current directory)" },
	{ "CDUP", CDUP, ARGS, 1,	"(change to parent directory)" },
	{ "XCUP", CDUP, ARGS, 1,	"(change to parent directory)" },
	{ "STOU", STOU, STR1, 1,	"<sp> file-name" },
	{ "SIZE", SIZE, OSTR, 1,	"<sp> path-name" },
	{ "MDTM", MDTM, OSTR, 1,	"<sp> path-name" },
	{ NULL,   0,    0,    0,	0 }
};

struct tab sitetab[] = {
	{ "UMASK", UMASK, ARGS, 1,	"[ <sp> umask ]" },
	{ "IDLE", IDLE, ARGS, 1,	"[ <sp> maximum-idle-time ]" },
	{ "CHMOD", CHMOD, NSTR, 1,	"<sp> mode <sp> file-name" },
	{ "HELP", HELP, OSTR, 1,	"[ <sp> <string> ]" },
	{ NULL,   0,    0,    0,	0 }
};

static struct tab *
lookup(p, cmd)
	struct tab *p;
	char *cmd;
{

	for (; p->name != NULL; p++)
		if (strcmp(cmd, p->name) == 0)
			return (p);
	return (0);
}

#include <arpa/telnet.h>

/*
 * getline - a hacked up version of fgets to ignore TELNET escape codes.
 */
char *
telnet_fgets(char *s, int n, FILE *iop)
{
	int c;
	register char *cs;

	cs = s;
/* tmpline may contain saved command from urgent mode interruption */
	for (c = 0; tmpline[c] != '\0' && --n > 0; ++c) {
		*cs++ = tmpline[c];
		if (tmpline[c] == '\n') {
			*cs++ = '\0';
			if (debug)
				syslog(LOG_DEBUG, "command: %s", s);
			tmpline[0] = '\0';
			return(s);
		}
		if (c == 0)
			tmpline[0] = '\0';
	}
	while ((c = getc(iop)) != EOF) {
		c &= 0377;
		if (c == IAC) {
		    if ((c = getc(iop)) != EOF) {
			c &= 0377;
			switch (c) {
			case WILL:
			case WONT:
				c = getc(iop);
				printf("%c%c%c", IAC, DONT, 0377&c);
				(void) fflush(stdout);
				continue;
			case DO:
			case DONT:
				c = getc(iop);
				printf("%c%c%c", IAC, WONT, 0377&c);
				(void) fflush(stdout);
				continue;
			case IAC:
				break;
			default:
				continue;	/* ignore command */
			}
		    }
		}
		*cs++ = c;
		if (--n <= 0 || c == '\n')
			break;
	}
	if (c == EOF && cs == s)
	    return (NULL);
	*cs++ = '\0';
	if (debug) {
		if (!cred.guest && strncasecmp("pass ", s, 5) == 0) {
			/* Don't syslog passwords */
			syslog(LOG_DEBUG, "command: %.5s ???", s);
		} else {
			register char *cp;
			register int len;

			/* Don't syslog trailing CR-LF */
			len = strlen(s);
			cp = s + len - 1;
			while (cp >= s && (*cp == '\n' || *cp == '\r')) {
				--cp;
				--len;
			}
			syslog(LOG_DEBUG, "command: %.*s", len, s);
		}
	}
	return (s);
}

void
toolong(int signo)
{
  (void)signo;
	reply(421,
	    "Timeout (%d seconds): closing control connection.", timeout);
	if (logging)
		syslog(LOG_INFO, "User %s timed out after %d seconds",
		    (cred.name ? cred.name : "unknown"), timeout);
	dologout(1);
}

static int
yylex()
{
	static int cpos, state;
	char *cp, *cp2;
	struct tab *p;
	int n;
	char c;

	for (;;) {
		switch (state) {

		case CMD:
			(void) signal(SIGALRM, toolong);
			(void) alarm((unsigned) timeout);
			if (telnet_fgets(cbuf, sizeof(cbuf)-1, stdin) == NULL) {
				reply(221, "You could at least say goodbye.");
				dologout(0);
			}
			(void) alarm(0);
#ifdef HAVE_SETPROCTITLE
			if (strncasecmp(cbuf, "PASS", 4) != NULL)
				setproctitle("%s: %s", proctitle, cbuf);
#endif /* HAVE_SETPROCTITLE */
			if ((cp = strchr(cbuf, '\r'))) {
				*cp++ = '\n';
				*cp = '\0';
			}
			if ((cp = strpbrk(cbuf, " \n")))
				cpos = cp - cbuf;
			if (cpos == 0)
				cpos = 4;
			c = cbuf[cpos];
			cbuf[cpos] = '\0';
			upper(cbuf);
			p = lookup(cmdtab, cbuf);
			cbuf[cpos] = c;
			if (p != 0) {
				if (p->implemented == 0) {
					nack(p->name);
					longjmp(errcatch,0);
					/* NOTREACHED */
				}
				state = p->state;
				yylval.s = p->name;
				return (p->token);
			}
			break;

		case SITECMD:
			if (cbuf[cpos] == ' ') {
				cpos++;
				return (SP);
			}
			cp = &cbuf[cpos];
			if ((cp2 = strpbrk(cp, " \n")))
				cpos = cp2 - cbuf;
			c = cbuf[cpos];
			cbuf[cpos] = '\0';
			upper(cp);
			p = lookup(sitetab, cp);
			cbuf[cpos] = c;
			if (p != 0) {
				if (p->implemented == 0) {
					state = CMD;
					nack(p->name);
					longjmp(errcatch,0);
					/* NOTREACHED */
				}
				state = p->state;
				yylval.s = p->name;
				return (p->token);
			}
			state = CMD;
			break;

		case OSTR:
			if (cbuf[cpos] == '\n') {
				state = CMD;
				return (CRLF);
			}
			/* FALLTHROUGH */

		case STR1:
		case ZSTR1:
		dostr1:
			if (cbuf[cpos] == ' ') {
				cpos++;
				state = state == OSTR ? STR2 : ++state;
				return (SP);
			}
			break;

		case ZSTR2:
			if (cbuf[cpos] == '\n') {
				state = CMD;
				return (CRLF);
			}
			/* FALLTHROUGH */

		case STR2:
			cp = &cbuf[cpos];
			n = strlen(cp);
			cpos += n - 1;
			/*
			 * Make sure the string is nonempty and \n terminated.
			 */
			if (n > 1 && cbuf[cpos] == '\n') {
				cbuf[cpos] = '\0';
				yylval.s = copy(cp);
				cbuf[cpos] = '\n';
				state = ARGS;
				return (STRING);
			}
			break;

		case NSTR:
			if (cbuf[cpos] == ' ') {
				cpos++;
				return (SP);
			}
			if (isdigit(cbuf[cpos])) {
				cp = &cbuf[cpos];
				while (isdigit(cbuf[++cpos]))
					;
				c = cbuf[cpos];
				cbuf[cpos] = '\0';
				yylval.i = atoi(cp);
				cbuf[cpos] = c;
				state = STR1;
				return (NUMBER);
			}
			state = STR1;
			goto dostr1;

		case ARGS:
			if (isdigit(cbuf[cpos])) {
				cp = &cbuf[cpos];
				while (isdigit(cbuf[++cpos]))
					;
				c = cbuf[cpos];
				cbuf[cpos] = '\0';
				yylval.i = atoi(cp);
				cbuf[cpos] = c;
				return (NUMBER);
			}
			switch (cbuf[cpos++]) {

			case '\n':
				state = CMD;
				return (CRLF);

			case ' ':
				return (SP);

			case ',':
				return (COMMA);

			case 'A':
			case 'a':
				return (A);

			case 'B':
			case 'b':
				return (B);

			case 'C':
			case 'c':
				return (C);

			case 'E':
			case 'e':
				return (E);

			case 'F':
			case 'f':
				return (F);

			case 'I':
			case 'i':
				return (I);

			case 'L':
			case 'l':
				return (L);

			case 'N':
			case 'n':
				return (N);

			case 'P':
			case 'p':
				return (P);

			case 'R':
			case 'r':
				return (R);

			case 'S':
			case 's':
				return (S);

			case 'T':
			case 't':
				return (T);

			}
			break;

		default:
			fatal("Unknown state in scanner.");
		}
		yyerror((char *) 0);
		state = CMD;
		longjmp(errcatch,0);
	}
}

void
upper(char *s)
{
	while (*s != '\0') {
		if (islower(*s))
			*s = toupper(*s);
		s++;
	}
}

static char *
copy(char *s)
{
	char *p;

	p = malloc((unsigned) strlen(s) + 1);
	if (p == NULL)
		fatal("Ran out of memory.");
	(void) strcpy(p, s);
	return (p);
}

static void
help(struct tab *ctab, char *s)
{
	struct tab *c;
	int width, NCMDS;
	const char *help_type;

	if (ctab == sitetab)
		help_type = "SITE ";
	else
		help_type = "";
	width = 0, NCMDS = 0;
	for (c = ctab; c->name != NULL; c++) {
		int len = strlen(c->name);

		if (len > width)
			width = len;
		NCMDS++;
	}
	width = (width + 8) &~ 7;
	if (s == 0) {
		int i, j, w;
		int columns, lines;

		lreply(214, "The following %scommands are recognized %s.",
		    help_type, "(* =>'s unimplemented)");
		columns = 76 / width;
		if (columns == 0)
			columns = 1;
		lines = (NCMDS + columns - 1) / columns;
		for (i = 0; i < lines; i++) {
			printf("   ");
			for (j = 0; j < columns; j++) {
				c = ctab + j * lines + i;
				printf("%s%c", c->name,
					c->implemented ? ' ' : '*');
				if (c + lines >= &ctab[NCMDS])
					break;
				w = strlen(c->name) + 1;
				while (w < width) {
					putchar(' ');
					w++;
				}
			}
			printf("\r\n");
		}
		(void) fflush(stdout);
		reply(214, "Direct comments to ftp-bugs@%s.", hostname);
		return;
	}
	upper(s);
	c = lookup(ctab, s);
	if (c == (struct tab *)0) {
		reply(502, "Unknown command %s.", s);
		return;
	}
	if (c->implemented)
		reply(214, "Syntax: %s%s %s", help_type, c->name, c->help);
	else
		reply(214, "%s%-*s\t%s; unimplemented.", help_type, width,
		    c->name, c->help);
}

static void
sizecmd(char *filename)
{
	switch (type) {
	case TYPE_L:
	case TYPE_I: {
		struct stat stbuf;
		if (stat(filename, &stbuf) < 0 || !S_ISREG(stbuf.st_mode))
			reply(550, "%s: not a plain file.", filename);
		else
			reply(213,
			      (sizeof (stbuf.st_size) > sizeof(long)
			       ? "%qu" : "%lu"), stbuf.st_size);
		break; }
	case TYPE_A: {
		FILE *fin;
		int c;
		off_t count;
		struct stat stbuf;
		fin = fopen(filename, "r");
		if (fin == NULL) {
			perror_reply(550, filename);
			return;
		}
		if (fstat(fileno(fin), &stbuf) < 0 || !S_ISREG(stbuf.st_mode)) {
			reply(550, "%s: not a plain file.", filename);
			(void) fclose(fin);
			return;
		}

		count = 0;
		while((c=getc(fin)) != EOF) {
			if (c == '\n')	/* will get expanded to \r\n */
				count++;
			count++;
		}
		(void) fclose(fin);

		reply(213, sizeof(count) > sizeof(long) ? "%qd" : "%ld",
		      count);
		break; }
	default:
		reply(504, "SIZE not implemented for Type %c.", "?AEIL"[type]);
	}
}

/* ARGSUSED */
static void
yyerror(const char *s)
{
  char *cp;

  (void)s;
  cp = strchr(cbuf,'\n');
  if (cp != NULL)
    *cp = '\0';
  reply(500, "'%s': command not understood.", cbuf);
}

