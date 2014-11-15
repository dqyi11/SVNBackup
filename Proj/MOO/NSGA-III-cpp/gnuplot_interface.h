#include <string>
#include <stdio.h>

#include <string>
#include <cstddef>

// ----------------------------------------------------------------------
// Gnuplot
//
// This is just a very simple interface to call gnuplot in the program.
// Now it seems to work only under windows + visual studio.
// ----------------------------------------------------------------------

class Gnuplot
{
public:
    Gnuplot() ;
    ~Gnuplot();
	
	// prohibit copying (VS2012 does not support 'delete')	
	Gnuplot(const Gnuplot &);
	Gnuplot & operator=(const Gnuplot &);

	// send any command to gnuplot
	void operator ()(const std::string & command);

	void reset() { operator()("reset"); }
	void replot() { operator()("replot"); }
	void set_title(const std::string &title);

	void plot(const std::string &fname, std::size_t x, std::size_t y);
	void splot(const std::string &fname, std::size_t x, std::size_t y, std::size_t z);

protected:
    FILE *gnuplotpipe;
};
