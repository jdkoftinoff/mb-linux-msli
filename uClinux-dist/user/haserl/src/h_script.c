/*--------------------------------------------------------------------------
 * functions related to opening, tokenizing and parsing a haserl script
 * Copyright (c) 2006-2007   Nathan Angelacos (nangel@users.sourceforge.net)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 ------------------------------------------------------------------------- */

#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "common.h"
#include "h_error.h"
#include "h_script.h"
#include "h_bash.h"
#include "haserl.h"

/* HTML, RUN, INCLUDE, EVAL, COMMENT, NOOP }; */

const char *g_tag[] = {
  "",
  "",
  "in",
  "=",
  "#",
  ""
};

/* the open and close tags */
char open_tag[3] = "<%";
char close_tag[3] = "%>";


/* Open a script and return a populated script_t structure
 */
script_t *
load_script (char *filename, script_t * scriptlist)
{
  script_t *scriptbuf;
  int scriptfp;
  struct stat filestat;

  scriptfp = open (filename, O_NONBLOCK + O_RDONLY);
  if (scriptfp == -1)
    {				/* open failed */
      die_with_message (NULL, NULL, g_err_msg[E_FILE_OPEN_FAIL], filename);
    }

  fstat (scriptfp, &filestat);
  scriptbuf = (script_t *) xmalloc (sizeof (script_t));
  scriptbuf->name = (char *) xmalloc (strlen (filename) + 1);
  scriptbuf->buf = (char *) xmalloc (filestat.st_size + 1);

  memset (scriptbuf->name, 0, strlen (filename) + 1);
  memcpy (scriptbuf->name, filename, strlen (filename));
  memset (scriptbuf->buf, 0, filestat.st_size + 1);
  read (scriptfp, scriptbuf->buf, filestat.st_size);

  scriptbuf->size = filestat.st_size;
  scriptbuf->uid = filestat.st_uid;
  scriptbuf->gid = filestat.st_gid;
  scriptbuf->curpos = 0;
  scriptbuf->next = NULL;

  /* if we already have scripts, add this one to the end */
  if (scriptlist != NULL)
    {
      while (scriptlist->next)
	scriptlist = scriptlist->next;
      scriptlist->next = scriptbuf;
    }

  /* skip over the first line, if the file starts with a #!
   * This means that offset will start at the beginning of
   * the second line in most cases.
   */
  if (memcmp (scriptbuf->buf, "#!", 2) == 0)
    {
      while ((scriptbuf->curpos < scriptbuf->size) &&
	     ((char) scriptbuf->buf[scriptbuf->curpos] != '\n'))
	{
	  (scriptbuf->curpos)++;
	}
      (scriptbuf->curpos)++;
    }

  /* If this is the first script, switch to <? ?> mode only
   * if <% is not used anywhere else in the script.  Otherwise
   * don't change the tagging method
   */
  if (scriptlist == NULL)
    {
      if (strstr (scriptbuf->buf + scriptbuf->curpos, open_tag) == NULL)
	{
	  open_tag[1] = '?';
	  close_tag[0] = '?';
	}
    }
  close (scriptfp);
  return (scriptbuf);
}

/* Free the script structures */
void
free_script_list (script_t * script)
{
  script_t *next;
  while (script)
    {
      next = script->next;
      if (script->name)
	free (script->name);
      if (script->buf)
	free (script->buf);
      free (script);
      script = next;
    }
}

/* Insert this token into the token chain.
 * If tokenlist is null, create a new one
 */
token_t *
push_token_on_list (token_t * tokenlist, script_t * scriptbuf,
		    char *start, size_t len)
{
  token_t *me, *next;
  if (len == 0)
    return (tokenlist);

  me = (token_t *) xmalloc (sizeof (token_t));

  if (tokenlist == NULL)
    {
      next = NULL;
    }
  else
    {
      next = tokenlist->next;
      tokenlist->next = me;
    }

  me->next = next;
  me->script = scriptbuf;
  me->buf = start;
  me->len = len;

  return (me);
}

/* Free a token chain */
void
free_token_list (token_t * tokenlist)
{
  token_t *next;

  while (tokenlist)
    {
      next = tokenlist->next;
      free (tokenlist);
      tokenlist = next;
    }
}

#ifndef JUST_LUACSHELL

/* return the point in a script where the "comment" ends */
char *
skip_comment (char *startbuf, char *endbuf)
{
  unsigned int c_lev = 1;
  char *s_tag, *e_tag;

  startbuf += 2;
  while (startbuf < endbuf)
    {
      s_tag = strstr (startbuf, open_tag);
      e_tag = strstr (startbuf, close_tag);

      if (!e_tag)
	{
	  break;
	}

      if ((s_tag) && (s_tag < e_tag))
	{
	  c_lev++;
	  startbuf = s_tag + 2;
	  continue;
	}
      /* otherwise, we have an end tag first */
      c_lev--;
      startbuf = e_tag;
      if (c_lev == 0)
	{
	  return (startbuf);
	}
      startbuf += 2;

    }
  return NULL;
}

/* build a tokenchain from a script.   This step just
 * splits a script buf into parts that are separated
 * by <% %>.  If the <% %> are out of order, then it
 * flags that, but doesn't try to do anything else.
 *
 * For nesting comments, this function IS aware of 
 * the comment tag, and will try accomodate commented
 * tags.  Commented script never enters the token list
 */
token_t *
build_token_list (script_t * scriptbuf, token_t * tokenlist)
{

  char *start, *end, *curpos, *endpos;
  token_t *curtoken, *firsttoken;

  curtoken = tokenlist;
  firsttoken = tokenlist;

  curpos = scriptbuf->buf + scriptbuf->curpos;
  endpos = scriptbuf->buf + scriptbuf->size;

  while (curpos < endpos)
    {
      start = strstr (curpos, open_tag);
      end = strstr (curpos, close_tag);

      /* if this is a comment tag, the end is at the end of the comment */
      if ((start) && (memcmp (start + 2, g_tag[COMMENT], 1) == 0))
	{
	  /* save any bare html before the comment */
	  curtoken = push_token_on_list (curtoken, scriptbuf, curpos,
					 start - curpos);
	  if (firsttoken == NULL)
	    firsttoken = curtoken;
	  /* push start of token to end of token  */
	  end = skip_comment (start, endpos);
	  if (end)
	    {
	      curpos = end + 2;
	      continue;
	    }
	}

      if (start && !end)
	die_with_message (scriptbuf, start, g_err_msg[E_NO_END_MARKER],
			  open_tag[1]);

      if ((start > end) || (!start && end))
	die_with_message (scriptbuf, end, g_err_msg[E_END_BEFORE_BEGIN],
			  open_tag[1], open_tag[1]);

      if (start && (strstr (start + 1, open_tag)
		    && (strstr (start + 1, open_tag) < end)))
	die_with_message (scriptbuf, start, g_err_msg[E_NO_END_MARKER],
			  open_tag[1]);
      if (end)
	{
	  /* push curpos to the start of the token */
	  curtoken = push_token_on_list (curtoken, scriptbuf, curpos,
					 start - curpos);
	  if (firsttoken == NULL)
	    firsttoken = curtoken;
	  /* push start of token to end of token  */
	  curtoken =
	    push_token_on_list (curtoken, scriptbuf, start, end - start);
	  if (firsttoken == NULL)
	    firsttoken = curtoken;
	  curpos = end + 2;
	}
      else
	{
	  /* push curpos to end of script */
	  curtoken =
	    push_token_on_list (curtoken, scriptbuf, curpos, endpos - curpos);
	  if (firsttoken == NULL)
	    firsttoken = curtoken;
	  curpos = endpos;
	}
    }

  return (firsttoken);
}

/* syntax check a script */
void
preprocess_token_list (token_t * tokenlist)
{
  script_t *newscript;
  token_t *me;
  char *cp;

  me = tokenlist;
  /* walk the chain to fill in the tags */
  while (me)
    {
      if (memcmp (me->buf, open_tag, 2))
	{
	  me->tag = HTML;
	}
      else
	{
	  me->tag = NOOP;
	  me->buf[me->len] = '\0';
	  cp = me->buf + 2;	/* skip <? */
	  if (memcmp (cp, g_tag[INCLUDE], 2) == 0)
	    {
	      me->tag = INCLUDE;
	      /* skip the first word - in, include, include-file, etc */
	      me->buf = find_whitespace (me->buf);
	      me->buf = skip_whitespace (me->buf);
	      cp = find_whitespace (me->buf);
	      *cp = '\0';
	      me->len = strlen (me->buf) + 1;
	      newscript = load_script (me->buf, me->script);
	      build_token_list (newscript, me);
	    }
	  if (memcmp (cp, g_tag[EVAL], 1) == 0)
	    {
	      me->tag = EVAL;
	      me->buf = find_whitespace (me->buf);
	      me->len = strlen (me->buf);
	    }
	  if (isspace (*cp))
	    {
	      me->tag = RUN;
	      me->buf = cp;
	    }
	  if (me->tag == NOOP)
	    {
	      die_with_message (me->script, cp, g_err_msg[E_NO_OP]);
	    }
	  me->len = strlen (me->buf);
	}
      me = me->next;
    }
}


token_t *
process_token_list (buffer_t * buf, token_t * token)
{
  char *c;

  buffer_init (buf);
  shell_exec (buf, "\n");	/* try to get the error reporting match the line number */

  while (token)
    {
      switch (token->tag)
	{
	case HTML:
	  /* Change from 0.8.0  - if the whole thing is just
	     whitespace, don't print it */
	  c = token->buf;
	  while ((c < (token->buf + token->len)) && (isspace (*c)))
	    c++;
	  if (c != token->buf + token->len)
	    shell_echo (buf, token->buf, token->len);
	  break;
	case RUN:
	  shell_exec (buf, token->buf);
	  shell_exec (buf, "\n");
	  break;
	case EVAL:
	  shell_eval (buf, token->buf, token->len);
	  break;
	default:
	  break;
	}
      token = token->next;
    }

  return (token);

}

#endif
