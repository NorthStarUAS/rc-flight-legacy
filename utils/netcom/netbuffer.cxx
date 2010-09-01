/*
     PLIB - A Suite of Portable Game Libraries
     Copyright (C) 1998,2002  Steve Baker
 
     This library is free software; you can redistribute it and/or
     modify it under the terms of the GNU Library General Public
     License as published by the Free Software Foundation; either
     version 2 of the License, or (at your option) any later version.
 
     This library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
     Library General Public License for more details.
 
     You should have received a copy of the GNU Library General Public
     License along with this library; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 
     For further information visit http://plib.sourceforge.net

     $Id: netBuffer.cxx 1568 2002-09-02 06:05:49Z sjbaker $
*/

#include "netbuffer.hxx"

netBuffer netBufferChannel::in_buffer;
netBuffer netBufferChannel::out_buffer;
int netBufferChannel::connection_count;

// FIXME: we need to think about how to handle buffer over runs from
// the sending side.  The critical thing is we do not want the
// incoming buffer at the network layer to fill up so the sending side
// blocks (is that possible?).  Consider reading all available data
// from the network interface and just discarding what we don't have
// room for in our input buffer.

void
netBufferChannel::handleRead (void)
{
  int max_read = in_buffer.getMaxLength() - in_buffer.getLength() ;
  if (max_read)
  {
    char* data = in_buffer.getData() + in_buffer.getLength() ;
    int num_read = recv (data, max_read) ;
    if (num_read > 0)
    {
      in_buffer.append (num_read) ;
      //ulSetError ( UL_DEBUG, "netBufferChannel: %d read", num_read ) ;
    }
  }
  if (in_buffer.getLength())
  {
    handleBufferRead (in_buffer);
  }
}

void
netBufferChannel::handleWrite (void)
{
  if (out_buffer.getLength())
  {
    if (isConnected())
    {
      int length = out_buffer.getLength() ;
      if (length>512)
        length=512;
      int num_sent = netChannel::send (
        out_buffer.getData(), length);
      if (num_sent > 0)
      {
        out_buffer.remove (0, num_sent);
        //ulSetError ( UL_DEBUG, "netBufferChannel: %d sent", num_sent ) ;
      }
    }
  }
  else if (should_close)
  {
    close();
  }
}


/**
 * 
 */
void
netBufferChannel::handleAccept()
{
    netAddress addr;
    int handle = netChannel::accept( &addr );
    printf("Telnet server accepted connection from %s:%d\n",
           addr.getHost(), addr.getPort() );
    netBufferChannel* channel = new netBufferChannel();
    channel->setHandle( handle );
    if ( connection_count > 1 ) {
	const char *msg = "\nThere is already an active connection, closing!\n\n";
	int len = strlen(msg);
	channel->bufferSend(msg, len);
	channel->poll();
	channel->close();
    }
}
