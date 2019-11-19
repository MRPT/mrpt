/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/****************************************************************************** 
 * 
 *  file:  VersionVisitor.h
 * 
 *  Copyright (c) 2003, Michael E. Smoot .
 *  All rights reverved.
 * 
 *  See the file COPYING in the top directory of this distribution for
 *  more information.
 *  
 *  THE SOFTWARE IS PROVIDED _AS IS_, WITHOUT WARRANTY OF ANY KIND, EXPRESS 
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 *  DEALINGS IN THE SOFTWARE.  
 *  
 *****************************************************************************/ 


#pragma once

#include <mrpt/3rdparty/tclap/CmdLineInterface.h>
#include <mrpt/3rdparty/tclap/CmdLineOutput.h>
#include <mrpt/3rdparty/tclap/Visitor.h>

namespace TCLAP {

/**
 * A Vistor that will call the version method of the given CmdLineOutput
 * for the specified CmdLine object and then exit.
 */
class VersionVisitor: public Visitor
{
	protected:

		/**
		 * The CmdLine of interest.
		 */
		CmdLineInterface* _cmd;

		/**
		 * The output object. 
		 */
		CmdLineOutput** _out;

	public:

		/**
		 * Constructor.
		 * \param cmd - The CmdLine the output is generated for. 
		 * \param out - The type of output. 
		 */
		VersionVisitor( CmdLineInterface* cmd, CmdLineOutput** out ) 
				: Visitor(), _cmd( cmd ), _out( out ) { }

		/**
		 * Calls the version method of the output object using the
		 * specified CmdLine.
		 */
		void visit() override { (*_out)->version(*_cmd); throw ActionDoneException(); }

};

}

