import	java.io.*;
import	java.util.*;
import java.util.regex.Pattern;
import java.util.regex.Matcher;
import	java.lang.*;

/**
 * A class to simplify symbolic transform expressions.
 * @author Peter I. Corke peter.i.corke@gmail.com
 * 
 * Major update 2/2014
 */



public class DHFactor {

    ElementList  results;

    // Matlab callable constructor
	public DHFactor(String src) {
        try {
            results = parseString(src);
			System.out.println("In DHFactor, parseString is done"); // debug
        }
        catch (IllegalArgumentException e) {
            System.err.println(e.getMessage());
        }
        if (!this.isValid())
            System.out.println("DHFactor: error: Incomplete factorization, no DH equivalent found");
    }

    private String angle(Element e) {
        return angle(e.constant);
    }


    public String toString() {
        return results.toString();
    }

    /*
    public String display() {
        return results.toString();
    }
    */

    private String angle(int a)
    {
        if (a == 0)
            return "0";
        else if (a == 90)
            return "pi/2";
        else if (a == -90)
            return "-pi/2";
        else
			throw new IllegalArgumentException("bad transform angle");
    }

    private String el2matlab(int from, int to)
    {
        String  xform = "";
        int     i;

        for (i=from; i<to; i++) {
            Element e = results.get(i);

            if (xform.length() > 0)
                xform += "*";

            switch (e.type) {
            case Element.RX:    xform += "trotx(" + angle(e) + ")"; break;
            case Element.RY:    xform += "troty(" + angle(e) + ")"; break;
            case Element.RZ:    xform += "trotz(" + angle(e) + ")"; break;
            case Element.TX:    xform += "transl(" + e.symconst + ",0,0)"; break;
            case Element.TY:    xform += "transl(0, " + e.symconst + ",0)"; break;
            case Element.TZ:    xform += "transl(0,0," + e.symconst + ")"; break;
            }
        }
        if (xform.length() == 0)
            xform = "eye(4,4)";
        return xform;
    }

    /*
     * Create a Toolbox legacy DH matrix. The column order is:
     *
     *      theta d a alpha
     */
    public String dh() {
		String	s = "[";
        String  theta, d;

		for (int i=0; i<results.size(); i++) {
			Element e = results.get(i);
            if (e.type == Element.DH_STANDARD) {
                // build up the string: theta d a alpha
                if (e.prismatic == 1) {
                    // prismatic joint
                    d = "0";    // by definition
                    theta = angle(e.theta);
                } else {
                    // revolute joint
                    theta = "0";    // by definition
                    d = (e.D == null) ? "0" : e.D;
                }

                s += theta; s += ", ";
                s += d; s += ", ";
                s += (e.A == null) ? "0" : e.A; s += ", ";
                s += angle(e.alpha);
                s += ", " + e.prismatic;
                s += "; ";
            };
        }
        s += "]";
		return s;
	}

    /*
     * Check the transform string is valid
     *
     */
    public boolean isValid() {
        Element e;
        int iprev = -1;

		for (int i=0; i<results.size(); i++) {
			e = results.get(i);
            if (e.type == Element.DH_STANDARD) {
                if (iprev >= 0) {
                    // we've seen a DH factor before
                    if ((i-iprev) > 1) {
                        // but it was too long ago, fail!
                        return false;
                    }
                }
                iprev = i;  // note where we saw it
            };
        }
		return true;
	}

    public String offset() {
		String	s = "[";
        Element e;

		for (int i=0; i<results.size(); i++) {
			e = results.get(i);
            if (e.type == Element.DH_STANDARD) {
                    s += angle(e.offset)  + " ";
            };
        }
        s += "]";
		return s;
	}

    // return base transform string in Matlab Toolbox form
    public String base() {
        int i;

        for (i=0; i<results.size(); i++) {
            Element e = results.get(i);

            if ( (e.type == Element.DH_STANDARD) || (e.type == Element.DH_MODIFIED) )
                return el2matlab(0, i);
        }

        return "eye(4,4)";
    }

    // return base transform string in Matlab Toolbox form
    public String tool() {
        int i;

        for (i=results.size()-1; i>=0; i--) {
            Element e = results.get(i);

            if ( (e.type == Element.DH_STANDARD) || (e.type == Element.DH_MODIFIED) )
                return el2matlab(i, results.size());
        }

        return "eye(4,4)";
    }


    // return Matlab Toolbox robot creation command
    public String command(String name) {
        if (this.isValid())
            return "SerialLink(" + this.dh() + ", 'name', '" + name +
                "', 'base', " + this.base() +
                ", 'tool', " + this.tool() +
                ", 'offset', " + this.offset() + ")";
        else
            return "error('incompletely factored transform string')";
    }

	public static ElementList parseFile(String filename) {
		BufferedReader	src;
		String			buffer;

		try {
            File file = new File(filename);

            if (!file.canRead() || !file.isFile())
                throw new IOException("dh: file access/type error");

            src = new BufferedReader(new FileReader(file));

            // read the file and parse it
            src = new BufferedReader(new FileReader(file));
            buffer = src.readLine();

        return parseString(buffer);
		}
		catch (FileNotFoundException e) {
			System.err.println(e.getMessage());
			System.exit(1);
		}
		catch (IOException e) {
			System.err.println(e.getMessage());
			System.exit(1);
		}
        return null;
    }

	public static ElementList parseStdin() {
		BufferedReader	src;
		String			buffer;

		try {
            // read the file and parse it
            src = new BufferedReader(new InputStreamReader(System.in));
            buffer = src.readLine();

        return parseString(buffer);
		}
		catch (IOException e) {
			System.err.println(e.getMessage());
			System.exit(1);
		}
        return null;
    }

	public static ElementList parseString(String buffer) {
		ElementList l = new ElementList();


		System.out.println("INIT: " + buffer);
		
		// each token is [R|T][x|y|z](arg)
		Pattern pattern = Pattern.compile("([RT][xyz]\\([^)]+\\))");
		Matcher tokens = pattern.matcher(buffer);

		while (tokens.find())
			l.add( new Element(tokens.group(1)) );

		System.out.println("PARSED: " + l);

		l.simplify();
		System.out.println(l);

		l.factorize(Element.DH_STANDARD, 0);
		System.out.println(l);

        return l;
	}

    // command line instantiation
    //   dhfactor file
    //   dhfactor < stdin
	public static void main(String args[]) {

        if (args.length == 1) {

            ElementList l  = parseString(args[0]);
            System.err.println( l );
            
        } else {
            ElementList l  = parseStdin();
            System.err.println( l );
        }
    }
}
