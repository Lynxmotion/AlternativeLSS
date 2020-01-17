#pragma once


//#include <functional>
//#include <initializer_list>

const short LssNone = 0;
const short LssNoBroadcast = 1;
const short LssMatchAny = 2;

const short LssReply = 1;
const short LssNoReply = 0;
const short LssNoHandler = -1;
const short LssError = -2;

inline void* operator new (size_t size, void * ptr) noexcept { return ptr; }


template<class ... Args>
class LssPacketHandlers
{
  public:
    // All handler callbacks should have this prototype based on the template parameter-pack arguments
    using Callback = short(*)(LynxPacket&, Args...);

    // Flags can direct how a handler is matched against a packet
    using Flags = unsigned int;

    // This handler class holds packet match parameters and a callback to call on successful match
    class Handler {
    public:  
      LssCommands command;
      Flags  flags;
      Callback handler;

      Handler(LssCommands _command, Flags _flags, Callback _handler) 
        : command(_command), flags(_flags), handler(_handler) {
          if((command & LssCommandModes) ==0)
            command |= LssAction;
      }
      
    };

    // single constructor for defining packet handlers
    // Packet handler classes should be global and allocated once, therefor no copy or assignment constructors should be used
    template<size_t N>
    LssPacketHandlers(const Handler(&handlers)[N]) 
      : _handlers(nullptr), _count(N) {
        _handlers = (Handler*)calloc(N, sizeof(Handler));
        for(size_t i=0; i < _count; i++) {
          new (&_handlers[i]) Handler(handlers[i]);
        }
    }

    ~LssPacketHandlers() {
      if(_handlers) ::free(_handlers);
    }
   
    // if you are copying handlers you are doing it wrong :) 
    // we generate a compile error, took a look at the examples again to see how packet handlers should be constructed.
    LssPacketHandlers(const LssPacketHandlers& copy) = delete;
    LssPacketHandlers& operator=(const LssPacketHandlers& copy) = delete;

    // we should allow moves though
    LssPacketHandlers(LssPacketHandlers&& _move) : _handlers(_move._handlers), _count(_move._count) {
      _move._handlers = nullptr;
      _move._count = 0;
    }

    // assignment move operator
    LssPacketHandlers& operator=(LssPacketHandlers&& _move) {
      if(_handlers) ::free(_handlers);
      _handlers = _move._handlers;
      _count = _move._count;
      _move._handlers = nullptr;
      _move._count = 0;
      return *this;
    }

    // returns true if a packet is handled by the given callback handler
    inline static bool matches(const LynxPacket& p, Handler h) { 
      //return (match & rhs.match)==match && (!broadcast || rhs.broadcast); 
      LssCommands cmd = p.command & (LssCommandSet|LssUnits);
      LssCommands mode = p.command & LssCommandModes;
      if((mode & LssCommandModes) ==0)
        mode |= LssAction;    // todo: move this to the main handler so we only do it once
      bool command_match = (h.flags & LssMatchAny)
          ? (h.command & cmd)           // match ANY of the command bits
          : ((h.command & cmd)==cmd);   // match ALL of the command bits
      return command_match && (h.command & mode)>0 && (!p.broadcast() || (h.flags & LssNoBroadcast)==0);
    }

    // find and execute a handler for the given packet
    short operator()(LynxPacket& p, Args ... args) const {
      short r = LssNoHandler;
      for(size_t i=0; i < _count; i++) {
        if(matches(p, _handlers[i])) {
          r =  _handlers[i].handler(p, args...);
          if(r==LssError)
            return r;
        }
      }
      return r;
    }

  protected:
    Handler* _handlers;
    size_t _count;
};
