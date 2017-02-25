/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*
  This file is heavily based on the RPly C library by Greg Turk (February 1994)
  It was modified, C++'ized and STL'ized by J.L.Blanco for MRPT (Jan-Feb 2011)

  Below follows the original copyright notes by Greg Turk:
 ======================================================================
The interface routines for reading and writing PLY polygon files.

Greg Turk, February 1994

---------------------------------------------------------------

A PLY file contains a single polygonal _object_.

An object is composed of lists of _elements_.  Typical elements are
vertices, faces, edges and materials.

Each type of element for a given object has one or more _properties_
associated with the element type.  For instance, a vertex element may
have as properties the floating-point values x,y,z and the three unsigned
chars representing red, green and blue.

---------------------------------------------------------------

Copyright (c) 1994 The Board of Trustees of The Leland Stanford
Junior University.  All rights reserved.

Permission to use, copy, modify and distribute this software and its
documentation for any purpose is hereby granted without fee, provided
that the above copyright notice and this permission notice appear in
all copies of this software and that you do not sell the software.

THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,
EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY
WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.

*/

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/PLY_import_export.h>
#include <mrpt/system/string_utils.h>
#include <stdio.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;


#define PLY_ASCII      1        /* ascii PLY file */
#define PLY_BINARY_BE  2        /* binary PLY file, big endian */
#define PLY_BINARY_LE  3        /* binary PLY file, little endian */

//#define PLY_OKAY    0           /* ply routine worked okay */
//#define PLY_ERROR  -1           /* error in ply routine */

/* scalar data types supported by PLY format */

enum {
	PLY_START_TYPE = 0,
	PLY_CHAR      = 1,
	PLY_SHORT     = 2,
	PLY_INT       = 3,
	PLY_UCHAR     = 4,
	PLY_USHORT    = 5,
	PLY_UINT      = 6,
	PLY_FLOAT     = 7,
	PLY_DOUBLE    = 8,
	PLY_END_TYPE  = 9
};

//#define  PLY_SCALAR  0
//#define  PLY_LIST    1


const int NO_OTHER_PROPS = -1;

const char DONT_STORE_PROP = 0;
const char STORE_PROP      = 1;

const char OTHER_PROP = 0;
const char NAMED_PROP = 1;

typedef struct PlyProperty {    /* description of a property */

  std::string name;                           /* property name */
  int external_type;                    /* file's data type */
  int internal_type;                    /* program's data type */
  int offset;                           /* offset bytes of prop in a struct */

  int is_list;                          /* 1 = list, 0 = scalar */
  int count_external;                   /* file's count type */
  int count_internal;                   /* program's count type */
  int count_offset;                     /* offset byte for list count */

} PlyProperty;

typedef struct PlyElement
{     /* description of an element */
  PlyElement() :
     num(0),
	 size(0),
	 other_offset(NO_OTHER_PROPS)
  {
  }

  string name;                   /* element name */
  int num;                      /* number of elements in this object */
  int size;                     /* size of element (bytes) or -1 if variable */
  vector<PlyProperty> props;          /* list of properties in the file */
  vector<char>        store_prop;             /* flags: property wanted by user? */
  int other_offset;             /* offset to un-asked-for props, or -1 if none*/
  int other_size;               /* size of other_props structure */
} PlyElement;

struct PlyFile
{        /* description of PLY file */
	PlyFile(FILE* _fp=NULL) :
		fp(_fp),
		file_type(0),
		version(0),
		which_elem(NULL)
	{
	}


	FILE *fp;                     /* file pointer */
	int file_type;                /* ascii or binary */
	float version;                /* version number of file */
	vector<PlyElement> elems;           /* list of elements */
	vector<string> comments;              /* list of comments */
	vector<string> obj_info;              /* list of object info items */
	PlyElement *which_elem;       /* which element we're currently writing */
};


const std::string type_names[] = {
string("invalid"),
string("char"), string("short"), string("int"),
string("uchar"), string("ushort"), string("uint"),
string("float"), string("double"),
};
const int ply_type_size[] = {
  0, 1, 2, 4, 1, 2, 4, 4, 8
};


/* find an element in a plyfile's list */
PlyElement *find_element(PlyFile *, const std::string &s);

/* find a property in an element's list */
PlyProperty *find_property(PlyElement *, const std::string &s, int *);

/* write to a file the word describing a PLY file data type */
void write_scalar_type (FILE *, int);

/* read a line from a file and break it up into separate words */
vector<string> get_words(FILE *, string&);

/* write an item to a file */
void write_binary_item(FILE *, int, unsigned int, double, int);
void write_ascii_item(FILE *, int, unsigned int, double, int);

/* add information to a PLY file descriptor */
void add_element(PlyFile *, const vector<string> &);
void add_property(PlyFile *, const vector<string> &);
void add_comment(PlyFile *, const string &);
void add_obj_info(PlyFile *, const string &);

/* copy a property */
void copy_property(PlyProperty *, const PlyProperty *);

/* store a value into where a pointer and a type specify */
void store_item(char *, int, int, unsigned int, double);

/* return the value of a stored item */
void get_stored_item( void *, int, int *, unsigned int *, double *);

/* return the value stored in an item, given ptr to it and its type */
double get_item_value(const char *, int);

/* get binary or ascii item and store it according to ptr and type */
void get_ascii_item(const char *, int, int *, unsigned int *, double *);
int get_binary_item(FILE *, int, int, int *, unsigned int *, double *);  // return 0 on error

/* get a bunch of elements from a file */
void ascii_get_element(PlyFile *, char *);
void binary_get_element(PlyFile *, char *);


/*************/
/*  Writing  */
/*************/


/******************************************************************************
Given a file pointer, get ready to write PLY data to the file.

Entry:
  fp         - the given file pointer
  nelems     - number of elements in object
  elem_names - list of element names
  file_type  - file type, either ascii or binary

Exit:
  returns a pointer to a PlyFile, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *ply_write(
  FILE *fp,
  const vector<string> & elem_names,
  int file_type
)
{
  /* check for NULL file pointer */
  if (fp == NULL)
    return (NULL);

  /* create a record for this object */
  PlyFile *plyfile = new PlyFile(fp);

  plyfile->file_type = file_type;
  plyfile->version = 1.0;
  //plyfile->other_elems = NULL;

  /* tuck aside the names of the elements */

  plyfile->elems.resize( elem_names.size() );
  for (size_t i = 0; i < elem_names.size(); i++)
  {
    plyfile->elems[i].name = elem_names[i];
  }

  /* return pointer to the file descriptor */
  return (plyfile);
}


/******************************************************************************
Open a polygon file for writing.

Entry:
  filename   - name of file to read from
  nelems     - number of elements in object
  elem_names - list of element names
  file_type  - file type, either ascii or binary

Exit:
  version - version number of PLY file
  returns a file identifier, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *ply_open_for_writing(
  const char *name,
  const vector<string> &elem_names,
  int file_type,
  float *version
)
{
  PlyFile *plyfile;
  FILE *fp;

  /* open the file for writing */

  fp = fopen (name, "w");
  if (fp == NULL) {
    return (NULL);
  }

  /* create the actual PlyFile structure */

  plyfile = ply_write (fp, elem_names, file_type);
  if (plyfile == NULL)
    return (NULL);

  /* say what PLY file version number we're writing */
  *version = plyfile->version;

  /* return pointer to the file descriptor */
  return (plyfile);
}


/******************************************************************************
Describe an element, including its properties and how many will be written
to the file.

Entry:
  plyfile   - file identifier
  elem_name - name of element that information is being specified about
  nelems    - number of elements of this type to be written
  nprops    - number of properties contained in the element
  prop_list - list of properties
******************************************************************************/

void ply_describe_element(
  PlyFile *plyfile,
  const string &elem_name,
  int nelems,
  vector<PlyProperty> &prop_list
)
{
  /* look for appropriate element */
  PlyElement *elem = find_element (plyfile, elem_name);
  if (elem == NULL)
	  throw std::runtime_error(format("ply_describe_element: can't find element '%s'",elem_name.c_str()));

  elem->num = nelems;

  /* copy the list of properties */

  const size_t nprops = prop_list.size();
  elem->props.resize(nprops);
  elem->store_prop.resize(nprops);

  for (size_t i = 0; i < nprops; i++)
  {
    elem->props[i] = prop_list[i];
    elem->store_prop[i] = NAMED_PROP;
  }
}


/******************************************************************************
Describe a property of an element.

Entry:
  plyfile   - file identifier
  elem_name - name of element that information is being specified about
  prop      - the new property
******************************************************************************/

void ply_describe_property(
  PlyFile *plyfile,
  const char *elem_name,
  const PlyProperty *prop
)
{
  /* look for appropriate element */
  PlyElement *elem = find_element (plyfile, elem_name);
  if (elem == NULL) {
    fprintf(stderr, "ply_describe_property: can't find element '%s'\n",
            elem_name);
    return;
  }

  /* copy the new property */
  elem->props.push_back(*prop);
  elem->store_prop.push_back(NAMED_PROP);
}


/******************************************************************************
State how many of a given element will be written.

Entry:
  plyfile   - file identifier
  elem_name - name of element that information is being specified about
  nelems    - number of elements of this type to be written
******************************************************************************/

void ply_element_count(
  PlyFile *plyfile,
  const string &elem_name,
  int nelems
)
{
  //int i;
  PlyElement *elem;
  //PlyProperty *prop;

  /* look for appropriate element */
  elem = find_element (plyfile, elem_name);
  if (elem == NULL)
	  throw std::runtime_error(format("ply_element_count: can't find element '%s'",elem_name.c_str()));

  elem->num = nelems;
}


/******************************************************************************
Signal that we've described everything a PLY file's header and that the
header should be written to the file.

Entry:
  plyfile - file identifier
******************************************************************************/

void ply_header_complete(PlyFile *plyfile)
{
  FILE *fp = plyfile->fp;

  fprintf (fp, "ply\n");

  switch (plyfile->file_type) {
    case PLY_ASCII:
      fprintf (fp, "format ascii 1.0\n");
      break;
    case PLY_BINARY_BE:
      fprintf (fp, "format binary_big_endian 1.0\n");
      break;
    case PLY_BINARY_LE:
      fprintf (fp, "format binary_little_endian 1.0\n");
      break;
    default:
		throw std::runtime_error(format("ply_header_complete: bad file type = %d",plyfile->file_type));
  }

  /* write out the comments */

  for (size_t i = 0; i < plyfile->comments.size(); i++)
    fprintf (fp, "comment %s\n", plyfile->comments[i].c_str());

  /* write out object information */

  for (size_t i = 0; i < plyfile->obj_info.size(); i++)
    fprintf (fp, "obj_info %s\n", plyfile->obj_info[i].c_str());

  /* write out information about each element */

  for (size_t i = 0; i < plyfile->elems.size(); i++)
  {
    const PlyElement *elem = &plyfile->elems[i];
    fprintf (fp, "element %s %d\n", elem->name.c_str(), elem->num);

    /* write out each property */
    for (size_t j = 0; j < elem->props.size(); j++) {
      const PlyProperty *prop = &elem->props[j];
      if (prop->is_list) {
        fprintf (fp, "property list ");
        write_scalar_type (fp, prop->count_external);
        fprintf (fp, " ");
        write_scalar_type (fp, prop->external_type);
        fprintf (fp, " %s\n", prop->name.c_str());
      }
      else {
        fprintf (fp, "property ");
        write_scalar_type (fp, prop->external_type);
        fprintf (fp, " %s\n", prop->name.c_str());
      }
    }
  }

  fprintf (fp, "end_header\n");
}


/******************************************************************************
Specify which elements are going to be written.  This should be called
before a call to the routine ply_put_element().

Entry:
  plyfile   - file identifier
  elem_name - name of element we're talking about
******************************************************************************/

void ply_put_element_setup(PlyFile *plyfile, const string &elem_name)
{
  PlyElement *elem;

  elem = find_element (plyfile, elem_name);
  if (elem == NULL)
	  throw std::runtime_error(format("ply_elements_setup: can't find element '%s'", elem_name.c_str()));

  plyfile->which_elem = elem;
}


/******************************************************************************
Write an element to the file.  This routine assumes that we're
writing the type of element specified in the last call to the routine
ply_put_element_setup().

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to the element
******************************************************************************/

void ply_put_element(PlyFile *plyfile, void *elem_ptr)
{
  FILE *fp = plyfile->fp;
  char *elem_data,*item;
  char **item_ptr;
  int item_size;
  int int_val;
  unsigned int uint_val;
  double double_val;
  char **other_ptr;

  PlyElement *elem = plyfile->which_elem;
  elem_data = (char*)elem_ptr;
  other_ptr = (char **) (((char *) elem_ptr) + elem->other_offset);

  /* write out either to an ascii or binary file */

  if (plyfile->file_type == PLY_ASCII) {

    /* write an ascii file */

    /* write out each property of the element */
    for (size_t j = 0; j < elem->props.size(); j++)
	{
      const PlyProperty *prop = &elem->props[j];
      if (elem->store_prop[j] == OTHER_PROP)
        elem_data = *other_ptr;
      else
        elem_data = (char*)elem_ptr;
      if (prop->is_list) {
        item = elem_data + prop->count_offset;
        get_stored_item ((void *) item, prop->count_internal,
                         &int_val, &uint_val, &double_val);
        write_ascii_item (fp, int_val, uint_val, double_val,
                          prop->count_external);
        const size_t list_count = uint_val;
        item_ptr = (char **) (elem_data + prop->offset);
        item = item_ptr[0];
       item_size = ply_type_size[prop->internal_type];
        for (size_t k = 0; k < list_count; k++)
		{
          get_stored_item ((void *) item, prop->internal_type,
                           &int_val, &uint_val, &double_val);
          write_ascii_item (fp, int_val, uint_val, double_val,
                            prop->external_type);
          item += item_size;
        }
      }
      else {
        item = elem_data + prop->offset;
        get_stored_item ((void *) item, prop->internal_type,
                         &int_val, &uint_val, &double_val);
        write_ascii_item (fp, int_val, uint_val, double_val,
                          prop->external_type);
      }
    }

    fprintf (fp, "\n");
  }
  else {

    /* write a binary file */

    /* write out each property of the element */
    for (size_t j = 0; j < elem->props.size(); j++)
	{
      const PlyProperty *prop = &elem->props[j];
      if (elem->store_prop[j] == OTHER_PROP)
        elem_data = *other_ptr;
      else
        elem_data = (char*)elem_ptr;
      if (prop->is_list) {
        item = elem_data + prop->count_offset;
        item_size = ply_type_size[prop->count_internal];
        get_stored_item ((void *) item, prop->count_internal,
                         &int_val, &uint_val, &double_val);
        write_binary_item (fp, int_val, uint_val, double_val,
                           prop->count_external);
        const size_t list_count = uint_val;
        item_ptr = (char **) (elem_data + prop->offset);
        item = item_ptr[0];
        item_size = ply_type_size[prop->internal_type];
        for (size_t k = 0; k < list_count; k++) {
          get_stored_item ((void *) item, prop->internal_type,
                           &int_val, &uint_val, &double_val);
          write_binary_item (fp, int_val, uint_val, double_val,
                             prop->external_type);
          item += item_size;
        }
      }
      else {
        item = elem_data + prop->offset;
        item_size = ply_type_size[prop->internal_type];
        get_stored_item ((void *) item, prop->internal_type,
                         &int_val, &uint_val, &double_val);
        write_binary_item (fp, int_val, uint_val, double_val,
                           prop->external_type);
      }
    }

  }
}


/******************************************************************************
Specify a comment that will be written in the header.

Entry:
  plyfile - file identifier
  comment - the comment to be written
******************************************************************************/

void ply_put_comment(PlyFile *plyfile, const string &comment)
{
	plyfile->comments.push_back(comment);
}


/******************************************************************************
Specify a piece of object information (arbitrary text) that will be written
in the header.

Entry:
  plyfile  - file identifier
  obj_info - the text information to be written
******************************************************************************/

void ply_put_obj_info(PlyFile *plyfile, const string &obj_info)
{
	plyfile->obj_info.push_back(obj_info);
}




/*************/
/*  Reading  */
/*************/



/******************************************************************************
Given a file pointer, get ready to read PLY data from the file.

Entry:
  fp - the given file pointer

Exit:
  nelems     - number of elements in object
  elem_names - list of element names
  returns a pointer to a PlyFile, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *ply_read(FILE *fp, vector<string> &elem_names)
{
  //int found_format = 0;

  /* check for NULL file pointer */
  if (fp == NULL)
    return (NULL);

  /* create record for this object */
  PlyFile *plyfile = new PlyFile(fp);

  /* read and parse the file's header */
  string orig_line;
  vector<string> words = get_words (plyfile->fp, orig_line);

  if (words.empty() || words[0]!="ply" )
	return NULL;

  while (!words.empty()) {

    /* parse words */

    if (words[0]=="format")
    {
      if (words.size() != 3)
        return (NULL);
      if (words[1]=="ascii")
        plyfile->file_type = PLY_ASCII;
      else if (words[1]=="binary_big_endian")
        plyfile->file_type = PLY_BINARY_BE;
      else if (words[1]=="binary_little_endian")
        plyfile->file_type = PLY_BINARY_LE;
      else
        return (NULL);
      plyfile->version = atof (words[2].c_str());
      //found_format = 1;
    }
    else if (words[0]== "element")
      add_element (plyfile, words);
    else if (words[0]== "property")
      add_property (plyfile, words);
    else if (words[0]== "comment")
      add_comment (plyfile, orig_line);
    else if (words[0]== "obj_info")
      add_obj_info (plyfile, orig_line);
    else if (words[0]== "end_header")
      break;

	words = get_words (plyfile->fp, orig_line);
  }

  /* create tags for each property of each element, to be used */
  /* later to say whether or not to store each property for the user */

  for (size_t i = 0; i < plyfile->elems.size(); i++) {
    PlyElement *elem = &plyfile->elems[i];

	elem->store_prop.assign(elem->props.size(), DONT_STORE_PROP);
    elem->other_offset = NO_OTHER_PROPS; /* no "other" props by default */
  }

  /* set return values about the elements */
  elem_names.clear();
  for (size_t i = 0; i < plyfile->elems.size(); i++)
	elem_names.push_back(plyfile->elems[i].name);

  /* return a pointer to the file's information */

  return (plyfile);
}


/******************************************************************************
Open a polygon file for reading.

Entry:
  filename - name of file to read from

Exit:
  nelems     - number of elements in object
  elem_names - list of element names
  file_type  - file type, either ascii or binary
  version    - version number of PLY file
  returns a file identifier, used to refer to this file, or NULL if error
******************************************************************************/

PlyFile *ply_open_for_reading(
  const char *filename,
  vector<string> &elem_names,
  int *file_type,
  float *version
)
{
  FILE *fp;
  PlyFile *plyfile;

  /* open the file for reading */

  fp = fopen (filename, "r");
  if (fp == NULL)
    return (NULL);

  /* create the PlyFile data structure */

  plyfile = ply_read (fp, elem_names);

  /* determine the file type and version */
  if (plyfile)
  {
	  *file_type = plyfile->file_type;
	  *version = plyfile->version;
  }

  /* return a pointer to the file's information */

  return (plyfile);
}


/******************************************************************************
Get information about a particular element.

Entry:
  plyfile   - file identifier
  elem_name - name of element to get information about

Exit:
  nelems   - number of elements of this type in the file
  nprops   - number of properties
  returns a list of properties, or NULL if the file doesn't contain that elem
******************************************************************************/

vector<PlyProperty> ply_get_element_description(
  PlyFile *plyfile,
  const string &elem_name,
  int &nelems,
  int &nprops
)
{
  /* find information about the element */
  PlyElement *elem = find_element (plyfile, elem_name);
  if (elem == NULL)
    return vector<PlyProperty>();

  nelems = elem->num;
  nprops = elem->props.size();

  /* make a copy of the element's property list */
  return elem->props;
}



/******************************************************************************
Specify a property of an element that is to be returned.  This should be
called (usually multiple times) before a call to the routine ply_get_element().
This routine should be used in preference to the less flexible old routine
called ply_get_element_setup().

Entry:
  plyfile   - file identifier
  elem_name - which element we're talking about
  prop      - property to add to those that will be returned
******************************************************************************/

void ply_get_property(
  PlyFile *plyfile,
  const string &elem_name,
  const PlyProperty *prop
)
{
  PlyElement *elem;
  PlyProperty *prop_ptr;
  int index;

  /* find information about the element */
  elem = find_element (plyfile, elem_name);
  plyfile->which_elem = elem;

  /* deposit the property information into the element's description */

  prop_ptr = find_property (elem, prop->name, &index);
  if (prop_ptr == NULL) {
    fprintf (stderr, "Warning:  Can't find property '%s' in element '%s'\n",
             prop->name.c_str(), elem_name.c_str());
    return;
  }
  prop_ptr->internal_type  = prop->internal_type;
  prop_ptr->offset         = prop->offset;
  prop_ptr->count_internal = prop->count_internal;
  prop_ptr->count_offset   = prop->count_offset;

  /* specify that the user wants this property */
  elem->store_prop[index] = STORE_PROP;
}


/******************************************************************************
Read one element from the file.  This routine assumes that we're reading
the type of element specified in the last call to the routine
ply_get_element_setup().

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to location where the element information should be put
******************************************************************************/

void ply_get_element(PlyFile *plyfile, void *elem_ptr)
{
  if (plyfile->file_type == PLY_ASCII)
    ascii_get_element (plyfile, (char *) elem_ptr);
  else
    binary_get_element (plyfile, (char *) elem_ptr);
}


/******************************************************************************
Extract the comments from the header information of a PLY file.

Entry:
  plyfile - file identifier

Exit:
  num_comments - number of comments returned
  returns a pointer to a list of comments
******************************************************************************/

void ply_get_comments(PlyFile *plyfile, vector<string> &comments)
{
	comments = plyfile->comments;
}


/******************************************************************************
Extract the object information (arbitrary text) from the header information
of a PLY file.

Entry:
  plyfile - file identifier

Exit:
  num_obj_info - number of lines of text information returned
  returns a pointer to a list of object info lines
******************************************************************************/

void ply_get_obj_info(PlyFile *plyfile, vector<string> &obj_info)
{
	obj_info = plyfile->obj_info;
}


/*******************/
/*  Miscellaneous  */
/*******************/



/******************************************************************************
Close a PLY file.

Entry:
  plyfile - identifier of file to close
******************************************************************************/

void ply_close(PlyFile *plyfile)
{
  fclose (plyfile->fp);

  /* free up memory associated with the PLY file */
  delete plyfile;
}


/******************************************************************************
Get version number and file type of a PlyFile.

Entry:
  ply - pointer to PLY file

Exit:
  version - version of the file
  file_type - PLY_ASCII, PLY_BINARY_BE, or PLY_BINARY_LE
******************************************************************************/

void ply_get_info(PlyFile *ply, float *version, int *file_type)
{
  if (ply == NULL)
    return;

  *version = ply->version;
  *file_type = ply->file_type;
}



/******************************************************************************
Find an element from the element list of a given PLY object.

Entry:
  plyfile - file id for PLY file
  element - name of element we're looking for

Exit:
  returns the element, or NULL if not found
******************************************************************************/

PlyElement *find_element(PlyFile *plyfile, const string &element)
{
  for (size_t i = 0; i < plyfile->elems.size(); i++)
    if ( element == plyfile->elems[i].name)
      return &plyfile->elems[i];

  return (NULL);
}


/******************************************************************************
Find a property in the list of properties of a given element.

Entry:
  elem      - pointer to element in which we want to find the property
  prop_name - name of property to find

Exit:
  index - index to position in list
  returns a pointer to the property, or NULL if not found
******************************************************************************/

PlyProperty *find_property(PlyElement *elem, const std::string &prop_name, int *index)
{
  for (size_t i = 0; i < elem->props.size(); i++)
    if (string(prop_name)==elem->props[i].name ) {
      *index = i;
      return &elem->props[i];
    }
  *index = -1;
  return (NULL);
}


/******************************************************************************
Read an element from an ascii file.

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to element
******************************************************************************/

void ascii_get_element(PlyFile *plyfile, char *elem_ptr)
{
  int which_word;
  //FILE *fp = plyfile->fp;
  char *elem_data,*item=NULL;
  char *item_ptr;
  int item_size;
  int int_val;
  unsigned int uint_val;
  double double_val;
  int list_count;
  int store_it;
  char **store_array;
  char *other_data=NULL;
  int other_flag;

  /* the kind of element we're reading currently */
  PlyElement *elem = plyfile->which_elem;

  /* do we need to setup for other_props? */

  if (elem->other_offset != NO_OTHER_PROPS) {
    char **ptr;
    other_flag = 1;
    /* make room for other_props */
    other_data = (char *) malloc (elem->other_size);
    /* store pointer in user's structure to the other_props */
    ptr = (char **) (elem_ptr + elem->other_offset);
    *ptr = other_data;
  }
  else
    other_flag = 0;

  /* read in the element */
  string orig_line;
  vector<string> words = get_words (plyfile->fp, orig_line);

  if (words.empty())
	  throw std::runtime_error(format("ply_get_element: unexpected end of file"));

  which_word = 0;

  for (size_t j = 0; j < elem->props.size(); j++)
  {
    PlyProperty *prop = &elem->props[j];
    store_it = (elem->store_prop[j] | other_flag);

    /* store either in the user's structure or in other_props */
    if (elem->store_prop[j])
      elem_data = elem_ptr;
    else
      elem_data = other_data;

    if (prop->is_list) {       /* a list */

      /* get and store the number of items in the list */
      get_ascii_item (words[which_word++].c_str(), prop->count_external,
                      &int_val, &uint_val, &double_val);
      if (store_it) {
        item = elem_data + prop->count_offset;
        store_item(item, prop->count_internal, int_val, uint_val, double_val);
      }

      /* allocate space for an array of items and store a ptr to the array */
      list_count = int_val;
      item_size = ply_type_size[prop->internal_type];
      store_array = (char **) (elem_data + prop->offset);

      if (list_count == 0) {
        if (store_it)
          *store_array = NULL;
      }
      else {
        if (store_it) {
          item_ptr = (char *) malloc (sizeof (char) * item_size * list_count);
          item = item_ptr;
          *store_array = item_ptr;
        }

        /* read items and store them into the array */
        for (int k = 0; k < list_count; k++) {
          get_ascii_item (words[which_word++].c_str(), prop->external_type,
                          &int_val, &uint_val, &double_val);
          if (store_it) {
            store_item (item, prop->internal_type,
                        int_val, uint_val, double_val);
            item += item_size;
          }
        }
      }

    }
    else {                     /* not a list */
      get_ascii_item (words[which_word++].c_str(), prop->external_type,
                      &int_val, &uint_val, &double_val);
      if (store_it) {
        item = elem_data + prop->offset;
        store_item (item, prop->internal_type, int_val, uint_val, double_val);
      }
    }

  }

}


/******************************************************************************
Read an element from a binary file.

Entry:
  plyfile  - file identifier
  elem_ptr - pointer to an element
******************************************************************************/

void binary_get_element(PlyFile *plyfile, char *elem_ptr)
{
  FILE *fp = plyfile->fp;
  char *elem_data,*item=NULL;
  char *item_ptr;
  int item_size=0;
  int int_val;
  unsigned int uint_val;
  double double_val;
  int list_count;
  int store_it;
  char **store_array;
  char *other_data=NULL;
  int other_flag;

  int bin_file_type =  plyfile->file_type;

  /* the kind of element we're reading currently */
  PlyElement *elem = plyfile->which_elem;

  /* do we need to setup for other_props? */

  if (elem->other_offset != NO_OTHER_PROPS) {
    char **ptr;
    other_flag = 1;
    /* make room for other_props */
    other_data = (char *) malloc (elem->other_size);
    /* store pointer in user's structure to the other_props */
    ptr = (char **) (elem_ptr + elem->other_offset);
    *ptr = other_data;
  }
  else
    other_flag = 0;

  /* read in a number of elements */

  for (size_t j = 0; j < elem->props.size(); j++)
  {
    PlyProperty *prop = &elem->props[j];
    store_it = (elem->store_prop[j] | other_flag);

    /* store either in the user's structure or in other_props */
    if (elem->store_prop[j])
      elem_data = elem_ptr;
    else
      elem_data = other_data;

    if (prop->is_list) {       /* a list */

      /* get and store the number of items in the list */
      if (!get_binary_item (fp,bin_file_type, prop->count_external,
                      &int_val, &uint_val, &double_val))
      {
        // Error...
        fprintf(stderr,"RPly::binary_get_element: Error reading binary file!\n");
      }

      if (store_it) {
        item = elem_data + prop->count_offset;
        store_item(item, prop->count_internal, int_val, uint_val, double_val);
      }

      /* allocate space for an array of items and store a ptr to the array */
      list_count = int_val;
      /* The "if" was added by Afra Zomorodian 8/22/95
       * so that zipper won't crash reading plies that have additional
       * properties.
       */
      if (store_it) {
	item_size = ply_type_size[prop->internal_type];
      }
      store_array = (char **) (elem_data + prop->offset);
      if (list_count == 0) {
        if (store_it)
          *store_array = NULL;
      }
      else {
        if (store_it) {
          item_ptr = (char *) malloc (sizeof (char) * item_size * list_count);
          item = item_ptr;
          *store_array = item_ptr;
        }

        /* read items and store them into the array */
        for (int k = 0; k < list_count; k++) {
          if (!get_binary_item (fp, bin_file_type, prop->external_type,
                          &int_val, &uint_val, &double_val))
         {
           // Error...
           fprintf(stderr,"RPly::binary_get_element: Error reading binary file!\n");
         }

          if (store_it) {
            store_item (item, prop->internal_type,
                        int_val, uint_val, double_val);
            item += item_size;
          }
        }
      }

    }
    else {                     /* not a list */
      if (!get_binary_item (fp,bin_file_type, prop->external_type,
                      &int_val, &uint_val, &double_val))
      {
        // Error...
        fprintf(stderr,"RPly::binary_get_element: Error reading binary file!\n");
      }

      if (store_it) {
        item = elem_data + prop->offset;
        store_item (item, prop->internal_type, int_val, uint_val, double_val);
      }
    }

  }
}


/******************************************************************************
Write to a file the word that represents a PLY data type.

Entry:
  fp   - file pointer
  code - code for type
******************************************************************************/

void write_scalar_type (FILE *fp, int code)
{
  /* make sure this is a valid code */

  if (code <= PLY_START_TYPE || code >= PLY_END_TYPE)
	  throw std::runtime_error(format("write_scalar_type: bad data code = %d", code));

  /* write the code to a file */

  fprintf (fp, "%s", type_names[code].c_str());
}


/******************************************************************************
Get a text line from a file and break it up into words.

IMPORTANT: The calling routine call "free" on the returned pointer once
finished with it.

Entry:
  fp - file to read from

Exit:
  nwords    - number of words returned
  orig_line - the original line of characters
  returns a list of words from the line, or NULL if end-of-file
******************************************************************************/

vector<string> get_words(FILE *fp, string&orig_line)
{
#define BIG_STRING 4096
  char str[BIG_STRING];

  vector<string> words;

  ASSERT_(fp!=NULL)

  /* read in a line */
  char* result = fgets (str, BIG_STRING, fp);
  if (result == NULL) {
    orig_line = string();
    return words;
  }

  orig_line = string(str);
  mrpt::system::tokenize(orig_line," \t\r\n", words);

  return words;
}


/******************************************************************************
Return the value of an item, given a pointer to it and its type.

Entry:
  item - pointer to item
  type - data type that "item" points to

Exit:
  returns a double-precision float that contains the value of the item
******************************************************************************/

double get_item_value(char *item, int type)
{
  unsigned char *puchar;
  char *pchar;
  short int *pshort;
  unsigned short int *pushort;
  int *pint;
  unsigned int *puint;
  float *pfloat;
  double *pdouble;
  int int_value;
  unsigned int uint_value;
  double double_value;

  switch (type) {
    case PLY_CHAR:
      pchar = (char *) item;
      int_value = *pchar;
      return ((double) int_value);
    case PLY_UCHAR:
      puchar = (unsigned char *) item;
      int_value = *puchar;
      return ((double) int_value);
    case PLY_SHORT:
      pshort = (short int *) item;
      int_value = *pshort;
      return ((double) int_value);
    case PLY_USHORT:
      pushort = (unsigned short int *) item;
      int_value = *pushort;
      return ((double) int_value);
    case PLY_INT:
      pint = (int *) item;
      int_value = *pint;
      return ((double) int_value);
    case PLY_UINT:
      puint = (unsigned int *) item;
      uint_value = *puint;
      return ((double) uint_value);
    case PLY_FLOAT:
      pfloat = (float *) item;
      double_value = *pfloat;
      return (double_value);
    case PLY_DOUBLE:
      pdouble = (double *) item;
      double_value = *pdouble;
      return (double_value);
    default:
		throw std::runtime_error(format("get_item_value: bad type = %d", type));
  }
}


/******************************************************************************
Write out an item to a file as raw binary bytes.

Entry:
  fp         - file to write to
  int_val    - integer version of item
  uint_val   - unsigned integer version of item
  double_val - double-precision float version of item
  type       - data type to write out
******************************************************************************/

void write_binary_item(
  FILE *fp,
  int int_val,
  unsigned int uint_val,
  double double_val,
  int type
)
{
  unsigned char uchar_val;
  char char_val;
  unsigned short ushort_val;
  short short_val;
  float float_val;

  switch (type) {
    case PLY_CHAR:
      char_val = int_val;
      fwrite (&char_val, 1, 1, fp);
      break;
    case PLY_SHORT:
      short_val = int_val;
      fwrite (&short_val, 2, 1, fp);
      break;
    case PLY_INT:
      fwrite (&int_val, 4, 1, fp);
      break;
    case PLY_UCHAR:
      uchar_val = uint_val;
      fwrite (&uchar_val, 1, 1, fp);
      break;
    case PLY_USHORT:
      ushort_val = uint_val;
      fwrite (&ushort_val, 2, 1, fp);
      break;
    case PLY_UINT:
      fwrite (&uint_val, 4, 1, fp);
      break;
    case PLY_FLOAT:
      float_val = double_val;
      fwrite (&float_val, 4, 1, fp);
      break;
    case PLY_DOUBLE:
      fwrite (&double_val, 8, 1, fp);
      break;
    default:
		throw std::runtime_error(format("write_binary_item: bad type = %d", type));
  }
}


/******************************************************************************
Write out an item to a file as ascii characters.

Entry:
  fp         - file to write to
  int_val    - integer version of item
  uint_val   - unsigned integer version of item
  double_val - double-precision float version of item
  type       - data type to write out
******************************************************************************/

void write_ascii_item(
  FILE *fp,
  int int_val,
  unsigned int uint_val,
  double double_val,
  int type
)
{
  switch (type) {
    case PLY_CHAR:
    case PLY_SHORT:
    case PLY_INT:
      fprintf (fp, "%d ", int_val);
      break;
    case PLY_UCHAR:
    case PLY_USHORT:
    case PLY_UINT:
      fprintf (fp, "%u ", uint_val);
      break;
    case PLY_FLOAT:
    case PLY_DOUBLE:
      fprintf (fp, "%g ", double_val);
      break;
    default:
		throw std::runtime_error(format("write_ascii_item: bad type = %d", type));
  }
}




/******************************************************************************
Get the value of an item that is in memory, and place the result
into an integer, an unsigned integer and a double.

Entry:
  ptr  - pointer to the item
  type - data type supposedly in the item

Exit:
  int_val    - integer value
  uint_val   - unsigned integer value
  double_val - double-precision floating point value
******************************************************************************/

void get_stored_item(
  void *ptr,
  int type,
  int *int_val,
  unsigned int *uint_val,
  double *double_val
)
{
  switch (type) {
    case PLY_CHAR:
      *int_val = *((char *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case PLY_UCHAR:
      *uint_val = *((unsigned char *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case PLY_SHORT:
      *int_val = *((short int *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case PLY_USHORT:
      *uint_val = *((unsigned short int *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case PLY_INT:
      *int_val = *((int *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case PLY_UINT:
      *uint_val = *((unsigned int *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case PLY_FLOAT:
      *double_val = *((float *) ptr);
      *int_val = *double_val;
      *uint_val = *double_val;
      break;
    case PLY_DOUBLE:
      *double_val = *((double *) ptr);
      *int_val = *double_val;
      *uint_val = *double_val;
      break;
    default:
		throw std::runtime_error(format("get_stored_item: bad type = %d", type));
  }
}


/******************************************************************************
Get the value of an item from a binary file, and place the result
into an integer, an unsigned integer and a double.

Entry:
  fp   - file to get item from
  type - data type supposedly in the word

Exit:
  int_val    - integer value
  uint_val   - unsigned integer value
  double_val - double-precision floating point value

Return: 0: On error
******************************************************************************/

int get_binary_item(
  FILE *fp,
  int bin_file_type,
  int type,
  int *int_val,
  unsigned int *uint_val,
  double *double_val
)
{
  char c[8];
  void *ptr;

  ptr = (void *) c;

  switch (type) {
    case PLY_CHAR:
      if (fread (ptr, 1, 1, fp)!=1) return 0;
      *int_val = *((char *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case PLY_UCHAR:
      if (fread (ptr, 1, 1, fp)!=1) return 0;
      *uint_val = *((unsigned char *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case PLY_SHORT:
      if (fread (ptr, 2, 1, fp)!=1) return 0;
      *int_val = *((short int *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case PLY_USHORT:
      if (fread (ptr, 2, 1, fp)!=1) return 0;
      *uint_val = *((unsigned short int *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case PLY_INT:
      if (fread (ptr, 4, 1, fp)!=1) return 0;
      *int_val = *((int *) ptr);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;
    case PLY_UINT:
      if (fread (ptr, 4, 1, fp)!=1) return 0;
      *uint_val = *((unsigned int *) ptr);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;
    case PLY_FLOAT:
      if (fread (ptr, 4, 1, fp)!=1) return 0;
      *double_val = *((float *) ptr);
      *int_val = *double_val;
      *uint_val = *double_val;
      break;
    case PLY_DOUBLE:
      if (fread (ptr, 8, 1, fp)!=1) return 0;
      *double_val = *((double *) ptr);
      *int_val = *double_val;
      *uint_val = *double_val;
      break;
    default:
		throw std::runtime_error(format("get_binary_item: bad type = %d", type));
  }

  // Added by JL:
  // If the Big/Little endian format in the file is different than the native format, do the conversion:
#if MRPT_IS_BIG_ENDIAN
  const bool do_reverse = (bin_file_type==PLY_BINARY_LE);
#else
  const bool do_reverse = (bin_file_type==PLY_BINARY_BE);
#endif

  if (do_reverse)
  {
	int int_val2 = *int_val;
	unsigned int uint_val2 = *uint_val;
	double double_val2 = *double_val;
	mrpt::utils::reverseBytes(int_val2,*int_val);
	mrpt::utils::reverseBytes(uint_val2,*uint_val);
	mrpt::utils::reverseBytes(double_val2,*double_val);
  }

  return 1;
}


/******************************************************************************
Extract the value of an item from an ascii word, and place the result
into an integer, an unsigned integer and a double.

Entry:
  word - word to extract value from
  type - data type supposedly in the word

Exit:
  int_val    - integer value
  uint_val   - unsigned integer value
  double_val - double-precision floating point value
******************************************************************************/

void get_ascii_item(
  const char *word,
  int type,
  int *int_val,
  unsigned int *uint_val,
  double *double_val
)
{
  switch (type) {
    case PLY_CHAR:
    case PLY_UCHAR:
    case PLY_SHORT:
    case PLY_USHORT:
    case PLY_INT:
      *int_val = atoi (word);
      *uint_val = *int_val;
      *double_val = *int_val;
      break;

    case PLY_UINT:
      *uint_val = strtoul (word, (char **) NULL, 10);
      *int_val = *uint_val;
      *double_val = *uint_val;
      break;

    case PLY_FLOAT:
    case PLY_DOUBLE:
      *double_val = atof (word);
      *int_val = (int) *double_val;
      *uint_val = (unsigned int) *double_val;
      break;

    default:
		throw std::runtime_error(format("get_ascii_item: bad type = %d", type));
  }
}


/******************************************************************************
Store a value into a place being pointed to, guided by a data type.

Entry:
  item       - place to store value
  type       - data type
  int_val    - integer version of value
  uint_val   - unsigned integer version of value
  double_val - double version of value

Exit:
  item - pointer to stored value
******************************************************************************/

void store_item (
  char *item,
  int type,
  int int_val,
  unsigned int uint_val,
  double double_val
)
{
  unsigned char *puchar;
  short int *pshort;
  unsigned short int *pushort;
  int *pint;
  unsigned int *puint;
  float *pfloat;
  double *pdouble;

  switch (type) {
    case PLY_CHAR:
      *item = int_val;
      break;
    case PLY_UCHAR:
      puchar = (unsigned char *) item;
      *puchar = uint_val;
      break;
    case PLY_SHORT:
      pshort = (short *) item;
      *pshort = int_val;
      break;
    case PLY_USHORT:
      pushort = (unsigned short *) item;
      *pushort = uint_val;
      break;
    case PLY_INT:
      pint = (int *) item;
      *pint = int_val;
      break;
    case PLY_UINT:
      puint = (unsigned int *) item;
      *puint = uint_val;
      break;
    case PLY_FLOAT:
      pfloat = (float *) item;
      *pfloat = double_val;
      break;
    case PLY_DOUBLE:
      pdouble = (double *) item;
      *pdouble = double_val;
      break;
    default:
		throw std::runtime_error(format("store_item: bad type = %d", type));
  }
}


/******************************************************************************
Add an element to a PLY file descriptor.

Entry:
  plyfile - PLY file descriptor
  words   - list of words describing the element
  nwords  - number of words in the list
******************************************************************************/

void add_element (PlyFile *plyfile, const vector<string> &words)
{
  /* create the new element */
  plyfile->elems.push_back( PlyElement() );

  PlyElement *elem = &(*plyfile->elems.rbegin());
  elem->name = words[1];
  elem->num = atoi (words[2].c_str());
}


/******************************************************************************
Return the type of a property, given the name of the property.

Entry:
  name - name of property type

Exit:
  returns integer code for property, or 0 if not found
******************************************************************************/

int get_prop_type(const string &type_name)
{
  int i;

  for (i = PLY_START_TYPE + 1; i < PLY_END_TYPE; i++)
    if ( type_name == type_names[i])
      return (i);

  /* if we get here, we didn't find the type */
  return (0);
}


/******************************************************************************
Add a property to a PLY file descriptor.

Entry:
  plyfile - PLY file descriptor
  words   - list of words describing the property
  nwords  - number of words in the list
******************************************************************************/

void add_property (PlyFile *plyfile, const vector<string> &words)
{
  /* add this property to the list of properties of the current element */
  PlyElement *elem = &(*plyfile->elems.rbegin());

  elem->props.push_back( PlyProperty() );

  PlyProperty *prop = &(*elem->props.rbegin());


  /* create the new property */

  if (words[1]=="list") {       /* is a list */
    prop->count_external = get_prop_type (words[2]);
    prop->external_type = get_prop_type (words[3]);
    prop->name = words[4];
    prop->is_list = 1;
  }
  else {                                        /* not a list */
    prop->external_type = get_prop_type (words[1]);
    prop->name = words[2];
    prop->is_list = 0;
  }
}


/******************************************************************************
Add a comment to a PLY file descriptor.

Entry:
  plyfile - PLY file descriptor
  line    - line containing comment
******************************************************************************/

void add_comment (PlyFile *plyfile, const string &line)
{
  /* skip over "comment" and leading spaces and tabs */
  ply_put_comment (plyfile, mrpt::system::trim(line.substr(7)));
}


/******************************************************************************
Add a some object information to a PLY file descriptor.

Entry:
  plyfile - PLY file descriptor
  line    - line containing text info
******************************************************************************/

void add_obj_info (PlyFile *plyfile, const string &line)
{
  /* skip over "obj_info" and leading spaces and tabs */
  ply_put_obj_info (plyfile, mrpt::system::trim(line.substr(8)));
}


/******************************************************************************
Copy a property.
******************************************************************************/

void copy_property(PlyProperty *dest, const PlyProperty *src)
{
	*dest = *src;
}

const float VAL_NOT_SET = -1e10;

struct TVertex
{
	float x,y,z;
	float r,g,b;
	float intensity;
};

const PlyProperty vert_props[] = { /* list of property information for a vertex */
	//                                             is_list   count_external  count_internal   count_offset
	{"x", PLY_FLOAT, PLY_FLOAT, offsetof(TVertex,x), 0       , 0                 , 0              , 0},
	{"y", PLY_FLOAT, PLY_FLOAT, offsetof(TVertex,y), 0       , 0                 , 0              , 0},
	{"z", PLY_FLOAT, PLY_FLOAT, offsetof(TVertex,z), 0       , 0                 , 0              , 0},
	{"intensity", PLY_FLOAT, PLY_FLOAT, offsetof(TVertex,intensity), 0       , 0                 , 0              , 0}
};

struct TFace
{
  float intensity; /* this user attaches intensity to faces */
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
};

const PlyProperty face_props[] = { /* list of property information for a vertex */
  {"intensity", PLY_FLOAT, PLY_FLOAT, offsetof(TFace,intensity), 0, 0, 0, 0},
  {"vertex_indices", PLY_INT, PLY_INT, offsetof(TFace,verts), 1, PLY_UCHAR, PLY_UCHAR, offsetof(TFace,nverts)}
};


/*
		Loads from a PLY file.
*/
bool PLY_Importer::loadFromPlyFile(
	const std::string         &filename,
	CStringList  *file_comments,
	CStringList  *file_obj_info )
{
	try
	{
		/* open a PLY file for reading */
		vector<string> elist;  // element names
		int file_type;
		float version;
		PlyFile *ply = ply_open_for_reading(filename.c_str(), elist, &file_type, &version);

		/* go through each kind of element that we learned is in the file */
		/* and read them */

		for (size_t i = 0; i < elist.size(); i++)
		{
			/* get the description of the first element */
			const string &elem_name = elist[i];
			int num_elems=0, nprops=0;

			//vector<PlyProperty> plist =
			ply_get_element_description (ply, elem_name, num_elems, nprops);

			/* print the name of the element, for debugging */
			//printf ("element %s %d\n", elem_name, num_elems);

			/* if we're on vertex elements, read them in */
			if ("vertex"==elem_name)
			{
				/* set up for getting vertex elements */
				for (size_t k=0;k<sizeof(vert_props)/sizeof(PlyProperty);k++)
					ply_get_property (ply, elem_name, &vert_props[k]);

				/* grab all the vertex elements */
				this->PLY_import_set_vertex_count(num_elems);
				for (int j = 0; j < num_elems; j++)
				{
					TVertex  pt;
					pt.x = pt.y = pt.z = pt.r = pt.g = pt.b = pt.intensity = VAL_NOT_SET;

					/* grab an element from the file */
					ply_get_element (ply, reinterpret_cast<void*>(&pt));
					const TPoint3Df xyz(pt.x,pt.y,pt.z);
					if (pt.intensity!=VAL_NOT_SET)
					{	// Grayscale
						const TColorf col(pt.intensity,pt.intensity,pt.intensity);
						this->PLY_import_set_vertex(j,xyz, &col);
					}
					else
					if (pt.r!=VAL_NOT_SET && pt.g!=VAL_NOT_SET && pt.b!=VAL_NOT_SET)
					{	// RGB
						const TColorf col(pt.r,pt.g,pt.b);
						this->PLY_import_set_vertex(j,xyz, &col);
					}
					else
					{	// No color
						this->PLY_import_set_vertex(j,xyz);
					}
				}
			}

			// print out the properties we got, for debugging
			//for (int j = 0; j < nprops; j++)
			//	printf ("property %s\n", plist[j]->name);
		}

		// grab and print out the comments in the file
		if (file_comments)
		{
			vector<string> strs;
			ply_get_comments (ply, strs);
			*file_comments = CStringList(strs);
		}

		// grab and print out the object information
		if (file_obj_info)
		{
			vector<string> strs;
			ply_get_obj_info (ply, strs);
			*file_obj_info = CStringList(strs);
		}

		/* close the PLY file */
		ply_close (ply);

		// All OK:
		m_ply_import_last_error = std::string();
		return true;
	}
	catch(std::exception &e)
	{
		// Return error:
		m_ply_import_last_error = std::string(e.what());
		return false;
	}
}


bool PLY_Exporter::saveToPlyFile(
	const std::string  & filename,
	bool save_in_binary,
	const CStringList  & file_comments ,
	const CStringList  & file_obj_info  ) const
{
	try
	{
		/* list of the kinds of elements in the user's object */
		vector<string> elem_names;
		elem_names.push_back(string("vertex"));
		elem_names.push_back(string("face"));

		/* create the vertex index lists for the faces */
		//for (i = 0; i < nfaces; i++)
		//	faces[i].verts = vert_ptrs[i];

		/* open either a binary or ascii PLY file for writing */
		/* (the file will be called "test.ply" because the routines */
		/*  enforce the .ply filename extension) */

		float version;
		PlyFile *ply = ply_open_for_writing(
			filename.c_str(),
			elem_names,
			save_in_binary ?
#if MRPT_IS_BIG_ENDIAN
	PLY_BINARY_BE
#else
	PLY_BINARY_LE
#endif
				: PLY_ASCII,
			&version);

		/* describe what properties go into the vertex and face elements */

		const size_t nverts = this->PLY_export_get_vertex_count();
		const size_t nfaces = this->PLY_export_get_face_count();

		if (nverts)
		{
			// Find out if we have color:
			TPoint3Df pt;
			bool      pt_has_color;
			TColorf   pt_color;
			this->PLY_export_get_vertex(0,pt,pt_has_color,pt_color);

			ply_element_count (ply, "vertex", nverts);
			ply_describe_property (ply, "vertex", &vert_props[0]);  // x
			ply_describe_property (ply, "vertex", &vert_props[1]);  // y
			ply_describe_property (ply, "vertex", &vert_props[2]);  // z

			if (pt_has_color)
				ply_describe_property (ply, "vertex", &vert_props[3]);  // intensity
		}

		ply_element_count (ply, "face", nfaces);
		for (size_t k=0;k<sizeof(face_props)/sizeof(PlyProperty);k++)
			ply_describe_property (ply, "face", &face_props[k]);

		/* write a comment and an object information field */
		for (size_t k=0;k<file_comments.size();k++)
			ply_put_comment (ply, file_comments(k).c_str() );

		for (size_t k=0;k<file_obj_info.size();k++)
			ply_put_obj_info (ply, file_obj_info(k).c_str() );


		/* we have described exactly what we will put in the file, so */
		/* we are now done with the header info */
		ply_header_complete (ply);

		/* set up and write the vertex elements */
		ply_put_element_setup (ply, "vertex");
		for (size_t i = 0; i < nverts; i++)
		{
			TPoint3Df pt;
			bool      pt_has_color;
			TColorf   pt_color;
			this->PLY_export_get_vertex(i,pt,pt_has_color,pt_color);

			TVertex  ver;
			ver.x = pt.x;
			ver.y = pt.y;
			ver.z = pt.z;

			if (pt_has_color)
				ver.intensity = (1.0f/3.0f)*(pt_color.R+pt_color.G+pt_color.B);
			else
				ver.intensity = 0.5;

			ply_put_element (ply, (void *)&ver);
		}

		/* set up and write the face elements */
/*		ply_put_element_setup (ply, "face");
		for (size_t i = 0; i < nfaces; i++)
		{
			TFace face;
			this->PLY_export_get_face(...);
			ply_put_element (ply, (void *) &face);
		}*/

		/* close the PLY file */
		ply_close (ply);

		// All OK:
		m_ply_export_last_error = std::string();
		return true;
	}
	catch(std::exception &e)
	{
		// Return error:
		m_ply_export_last_error = std::string(e.what());
		return false;
	}
}
