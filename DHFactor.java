import	java.io.*;
import	java.util.*;
import	java.lang.*;

/**
 * A class to simplify symbolic transform expressions.
 * @author Peter I. Corke peter.i.corke@gmail.com
 */

/**
 * Element is a class that represents one element in a transform expression.
 * The transform types represented include:
 *		TX, TY, TZ	pure translation along X, Y and Z axes respectively
 *		RX, RY, RZ	pure rotations about the X, Y and Z axes respectively
 *		DH			Denavit-Hartenberg joint transformation
 *
 *	public boolean istrans()
 *	public boolean isrot()
 *	public int axis()
 *	public boolean isjoint() {
 *	public boolean factorMatch(int dhWhich, int i, int verbose) {
 *	public void add(Element e) {
 *	public Element(int type, int constant) {
 *	public Element(int type)	// new of specified type
 *
 *	public static String toString(Element [] e) {
 *	public String argString() {
 *	public String toString() {
 *
 * Constructors:
 *	Element(Element e) 	// clone of argument
 *	Element(Element e, int type, int sign) // clone of argument with new type
 *	Element(Element e, int type) // clone of argument with new type
 *	Element(String s)
 */
class Element {
    static final int    TX = 0,
                        TY = 1,
                        TZ = 2,
                        RX = 3,
                        RY = 4,
                        RZ = 5,
                        DH_STANDARD = 6,
                        DH_MODIFIED = 7;

    // one of TX, TY ... RZ, DH_STANDARD/MODIFIED
	int		type;

	// transform parameters, only one of these is set
	String	var;        // eg. q1, for joint var types
	String	symconst;   // eg. L1, for lengths
	int		constant;   // eg. 90, for angles

	// DH parameters, only set if type is DH_STANDARD/MODIFIED
	int 	theta,	
            alpha;
    String  A,
            D;
    int     prismatic;
    int     offset;

    // an array of counters for the application of each rule
    // just for debugging.
	static int[]	rules = new int[20];	// class variable

    // mapping from type to string
	static final String[] typeName = {
		"Tx", "Ty", "Tz", "Rx", "Ry", "Rz", "DH", "DHm"};

    // order of elementary transform for each DH convention
    // in each tuple, the first element is the transform type,
    // the second is true if it can be a joint variable.
	static final int dhStandard[][] = {
			{RZ, 1}, {TX, 0}, {TZ, 1}, {RX, 0} };
	static final int dhModified[][] = {
			{RX, 0}, {TX, 0}, {RZ, 1}, {TZ, 1} };

    /*
     * Display the number of times each rule was used in the
     * conversion.
     */
	public static void showRuleUsage() {
		for (int i=0; i<20; i++)
			if (rules[i] > 0)
				System.out.println("Rule " + i + ": " +
					rules[i]);
	}

    // test if the Element is a translation, eg. TX, TY or TZ
	public boolean istrans() {
		return (type == TX) || (type == TY) || (type == TZ);
	}

    // test if the Element is a rotation, eg. RX, RY or RZ
	public boolean isrot() {
		return (type == RX) || (type == RY) || (type == RZ);
	}

    // true if this transform represents a joint coordinate, ie. not
    // a constant
	public boolean isjoint() {
		return this.var != null;
	}

    // return the axis, 0 for X, 1 for Y, 2 for Z
	public int axis() {
		switch (type) {
		case TX:
		case RX:
			return 0;
		case TY:
		case RY:
			return 1;
		case TZ:
		case RZ:
			return 2;
		default:
			throw new IllegalArgumentException("bad transform type");
		}
	}

    // return the summation of two symbolic parameters as a string
	private String symAdd(String s1, String s2)
	{
		if ( (s1 == null) && (s2 == null) )
			return null;
		else if ( (s1 != null) && (s2 == null) )
			return new String(s1);
		else if ( (s1 == null) && (s2 != null) )
			return new String(s2);
		else {
			return s1 + "+" + s2;
		}
	}

    /**
     * Add the argument of another Element to this element.
     * assumes that variable has not already been set
     * used by factor() to build a DH element
     */
	public void add(Element e) {
		if ((this.type != DH_STANDARD) && (this.type != DH_MODIFIED))
			throw new IllegalArgumentException("wrong element type " + this);
		
		System.out.println("  adding: " + this + " += "  + e);
		switch (e.type) {
		case RZ:
            if (e.isjoint()) {
                this.prismatic = 0;
                this.var = e.var;
                this.offset = e.constant;
                this.theta = 0;
            } else
                this.theta = e.constant;
            break;
		case TX:
			this.A = e.symconst; break;
		case TZ:
            if (e.isjoint()) {
                this.prismatic = 1;
                this.var = e.var;
                this.D = null;
            } else
                this.D = e.symconst;
            break;
		case RX:
			this.alpha = e.constant; break;
		default:
			throw new IllegalArgumentException("cant factorize " + e);
		}
	}
    /*
	public void add(Element e) {
		if ((this.type != DH_STANDARD) && (this.type != DH_MODIFIED))
			throw new IllegalArgumentException("wrong element type " + this);
		
		System.out.println("  adding: " + this + " += "  + e);
		switch (e.type) {
		case RZ:
			this.theta = e.argString(); 
            if (e.isjoint())
                this.prismatic = 0;
            break;
		case TX:
			this.A = e.argString(); break;
		case TZ:
			this.D = e.argString(); 
            if (e.isjoint())
                this.prismatic = 1;
            break;
		case RX:
			this.alpha = e.argString(); break;
		default:
			throw new IllegalArgumentException("cant factorize " + e);
		}
	}
    */


    // test if this particular element could be part of a DH term
    //  eg. Rz(q1) can be, Rx(q1) cannot.
	public boolean factorMatch(int dhWhich, int i, int verbose) {
		int	dhFactors[][];
		boolean	match;

		switch (dhWhich) {
		case DH_STANDARD:
			dhFactors = dhStandard;
			break;
		case DH_MODIFIED:
			dhFactors = dhModified;
			break;
		default:
			throw new IllegalArgumentException("bad DH type");
		}

		match =	(this.type == dhFactors[i][0]) &&
			!((dhFactors[i][1] == 0) && this.isjoint());

		if (verbose > 0)
				System.out.println(" matching " + this + " (i=" + i + ") " + 
					" to " + typeName[dhFactors[i][0]] + "<" +
					dhFactors[i][1] + ">" + " -> " + match);
		return match;
	}

	/**
	 * test if two transforms can be merged
	 * @param e	the element to compare with this
	 * @return	- this if no merge to be done
	 *			- null if the result is a null transform
	 *			- new transform resulting from a merge.
	 */
	Element	merge(Element e) {
		
		/*
		 * don't merge if dissimilar transform or
		 * both are joint variables
		 */
		if (
			(e.type != this.type) ||
			(e.isjoint() && this.isjoint())
		) 
			return this;
		

		Element sum = new Element(this);

		sum.var = symAdd(this.var, e.var);
		sum.symconst = symAdd(this.symconst, e.symconst);
		sum.constant = this.constant + e.constant;
        if (Math.abs(sum.constant) > 90)
			throw new IllegalArgumentException("rotation angle > 90");

		/*
		 * remove a null transform which can result from
		 * a merge operation
		 */
		if ( !sum.isjoint() && (sum.symconst == null) && (sum.constant == 0)) {
			System.out.println("Eliminate: " + this + " " + e);
			return null;
		} else {
			System.out.println("Merge: " + this + " " + e + " := " + sum);
			return sum;
		}

	}

	/**
	 * test if two transforms need  to be swapped
	 * @param e	the element to compare with this
	 * @return	- true if swap is required
	 */
	boolean	swap(Element next, int dhWhich) {
		/*
		 * don't swap if both are joint variables
		 */
		if ( this.isjoint() && next.isjoint() )
			return false;

		switch (dhWhich) {
		case Element.DH_STANDARD:
			/*
			 * we want to sort terms into the order:
			 *	RZ
			 *	TX
			 *	TZ
			 *	RX
			 */
			/*                 TX TY TZ RX RY RZ		*/
			int order[] = {  2, 0, 3, 4, 0, 1 };
			if (
				((this.type == TZ) && (next.type == TX)) ||

				/*
				 * push constant translations through rotational joints
				 * of the same type
				 */
				((this.type == TX) && (next.type == RX) && next.isjoint()) ||
				((this.type == TY) && (next.type == RY)) && next.isjoint() ||
				((this.type == TZ) && (next.type == RZ)) && next.isjoint() ||

				(!this.isjoint() && (this.type == RX) && (next.type == TX)) ||
				(!this.isjoint() && (this.type == RY) && (next.type == TY)) ||
				//(!this.isjoint() && (this.type == RZ) && (next.type == TZ)) ||

				(!this.isjoint() && !next.isjoint() && (this.type == TZ) && (next.type == RZ)) ||

				/*
				 * move Ty terms to the right 
				 */
				((this.type == TY) && (next.type == TZ)) ||
				((this.type == TY) && (next.type == TX))
			) {
				System.out.println("Swap: " + this + " <-> " + next);
				return true;
			}
			break;
		case Element.DH_MODIFIED:
			if (
				((this.type == RX) && (next.type == TX)) ||
				((this.type == RY) && (next.type == TY)) ||
				((this.type == RZ) && (next.type == TZ)) ||
				((this.type == TZ) && (next.type == TX))
			) {
				System.out.println("Swap: " + this + " <-> " + next);
				return true;
			}
			break;
		default:
			throw new IllegalArgumentException("bad DH type");
		}
		return false;
	}
			
	/**
	 * Substitute this transform for a triple of transforms
     * that includes an RZ or TZ.
     *
	 * @return	- null if no substituion required
	 *			- array of Elements to substitute
	 */
	Element[] substituteToZ() {

		Element[] s = new Element[3];

		switch (this.type) {
		case RX:
			s[0] = new Element(RY, 90);
			s[1] = new Element(this, RZ);
			s[2] = new Element(RY, -90);
			return s;
		case RY:
			s[0] = new Element(RX, -90);
			s[1] = new Element(this, RZ);
			s[2] = new Element(RX, 90);
			return s;
		case TX:
			s[0] = new Element(RY, 90);
			s[1] = new Element(this, TZ);
			s[2] = new Element(RY, -90);
			return s;
		case TY:
			s[0] = new Element(RX, -90);
			s[1] = new Element(this, TZ);
			s[2] = new Element(RX, 90);
			return s;
		default:
			return null;
		}
	}

	Element[] substituteToZ(Element prev) {

		Element[] s = new Element[3];

		switch (this.type) {
		case RY:
			s[0] = new Element(RZ, 90);
			s[1] = new Element(this, RX);
			s[2] = new Element(RZ, -90);
			rules[8]++;
			return s;
		case TY:
			if (prev.type == RZ) {
				s[0] = new Element(RZ, 90);
				s[1] = new Element(this, TX);
				s[2] = new Element(RZ, -90);
				rules[6]++;
				return s;
			} else {
				s[0] = new Element(RX, -90);
				s[1] = new Element(this, TZ);
				s[2] = new Element(RX, 90);
				rules[7]++;
				return s;
			}
		default:
			return null;
		}
	}

	/**
	 * Simple rewriting rule for adjacent transform pairs.  Attempt to
	 * eliminate TY and RY.
	 * @param	previous element in list
	 * @return	- null if no substituion required
	 *			- array of Elements to subsitute
	 */
	Element[] substituteY(Element prev, Element next) {

		Element[] s = new Element[2];

		if (prev.isjoint() || this.isjoint())
			return null;

        /* note that if rotation is -90 we must make the displacement -ve */
		if ((prev.type == RX) && (this.type == TY)) {
				// RX.TY -> TZ.RX
				s[0] = new Element(this, TZ, prev.constant);
				s[1] = new Element(prev);
				rules[0]++;
				return s;
		} else if ((prev.type == RX) && (this.type == TZ)) {
				// RX.TZ -> TY.RX
				s[0] = new Element(this, TY, -prev.constant);
				s[1] = new Element(prev);
				rules[2]++;
				return s;
		} else if ((prev.type == RY) && (this.type == TX)) {
				// RY.TX-> TZ.RY
				s[0] = new Element(this, TZ, -prev.constant);
				s[1] = new Element(prev);
				rules[1]++;
				return s;
		} else if ((prev.type == RY) && (this.type == TZ)) {
				// RY.TZ-> TX.RY
				s[0] = new Element(this, TX, prev.constant);
				s[1] = new Element(prev);
				rules[11]++;
				return s;
		} else if ((prev.type == TY) && (this.type == RX)) {
				// TY.RX -> RX.TZ
				s[0] = new Element(this);
				s[1] = new Element(prev, TZ, -this.constant);
				rules[5]++;
				//return s;
				return null;
		} else if ((prev.type == RY) && (this.type == RX)) {
				// RY(Q).RX -> RX.RZ(-Q)
				s[0] = new Element(this);
				s[1] = new Element(prev, RZ, -1);
				rules[3]++;
				return s;
		} else if ((prev.type == RX) && (this.type == RY)) {
				// RX.RY -> RZ.RX
				s[0] = new Element(this, RZ);
				s[1] = new Element(prev);
				rules[4]++;
				return s;
		} else if ((prev.type == RZ) && (this.type == RX)) {
				// RZ.RX -> RX.RY
				s[0] = new Element(this);
				s[1] = new Element(prev, RY);
				//rules[10]++;
				//return s;
				return null;
		}
		return null;
	}

	/*
	 * Element contructors.  String is of the form:
	 */
	public Element(int type, int constant) {
		this.type = type;
		this.var = null;
		this.symconst = null;
		this.constant = constant;
	}

	public Element(int type) {		// new of specified type
		this.type = type;
	}

	public Element(Element e) {		// clone of argument
		this.type = e.type;
		if (e.var != null)
			this.var = new String(e.var);
		if (e.symconst != null)
			this.symconst = new String(e.symconst);
		this.constant = e.constant;
	}

	/**
	 * Constructor for Element.
	 * @param e	Template for new Element.
	 * @param type	Replacement type for new Element.
	 * @param sign	Sign of argument, either -1 or +1.
	 * @return a new Element with specified type and argument.
	 */
	public Element(Element e, int type, int sign) {	// clone of argument with new type
		this.type = type;
		if (e.var != null)
			this.var = new String(e.var);
        this.constant = e.constant;
		if (e.symconst != null)
            this.symconst = new String(e.symconst);

        if (sign < 0)
            this.negate();
	}

	public Element(Element e, int type) {	// clone of argument with new type
		this(e, type, 1);
	}

    // negate the arguments of the element
    public void negate() {
        //System.out.println("negate: " + this.constant + " " + this.symconst);

        // flip the numeric part, easy
		this.constant = -this.constant;


        if (this.symconst != null) {
            StringBuffer s = new StringBuffer(this.symconst);
            // if no leading sign character insert one (so we can flip it)
            if ((s.charAt(0) != '+') &&
                (s.charAt(0) != '-')
            )
                s.insert(0, '+');

            // go through the string and flip all sign chars
            for (int i=0; i<s.length(); i++)
                switch (s.charAt(i)) {
                case '+':
                    s.setCharAt(i, '-');
                    break;
                case '-':
                    s.setCharAt(i, '+');
                    break;
                default:
                    break;
                }
                
            this.symconst = new String(s);
       }
       //System.out.println("negate: " + this.constant + " " + this.symconst);
    }

	/**
	 * Parsing constructor.
	 * @param transform string expression, eg. Tx(q1)  Rx(90) Ty(L2)
	 *
	 * where q1 represents a joint variable, L2 is a dimension.
	 */
	public Element(String s)
				throws IllegalArgumentException {		// constructor
		int		i;
		String sType = s.substring(0,2);
		String sRest = s.substring(2);

		if (!(sRest.endsWith(")") && sRest.startsWith("(")))
			throw(new IllegalArgumentException("brackets"));

		for (i=0; i<6; i++)
			if (sType.equalsIgnoreCase(typeName[i]))
				break;
		if (i >= 6)
			throw(new IllegalArgumentException("bad transform name" + sType));
		type = i;

		sRest = sRest.substring(1, sRest.length()-1);
		switch (sRest.charAt(0)) {
		case 'q':
			var = sRest;
			break;
		case 'L':
			symconst = sRest;
			break;
		default:
			try {
				constant = Integer.parseInt(sRest);
			}
			catch(NumberFormatException e) {
				System.err.println(e.getMessage());
				throw(new IllegalArgumentException("bracket contents"));
			}
		}
	}

	// class method to convert Element vector to string
    /*
	public static String toString(Element [] e) {
		String 	s = "";

		for (int i=0; i<e.length; i++)
			s += e + " ";
		return s;
	}
    */

    /*
     * Return a string representation of the parameters (argument)
     * of the element, which can be a number, symbolic constant,
     * or a joint variable.
     */
	public String argString() {
		String s = "";

		switch (type) {
		case RX:
		case RY:
		case RZ:
		case TX:
		case TY:
		case TZ:
			if (var != null)
				s += var;
			if (symconst != null) {
				if (var != null)
					s += "+";
				s += symconst;
			}
			if (constant != 0)
				s += (constant < 0 ? "" : "+") + constant;
			break;
		case DH_STANDARD:
		case DH_MODIFIED:
            // theta, d, a, alpha

            // theta
            if (prismatic == 0) {
                s += var;
                if (offset > 0)
                    s += "+" + offset;
                 else if (offset < 0)
                    s += offset;
            } else
                s += theta;
			s += ", ";

            // d
            if (prismatic > 0)
                s += var;
            else
                s += (D == null) ? "0" : D;
			s += ", ";

            // a
			s += (A == null) ? "0" : A;
			s += ", ";

            // alpha
            s += alpha;
			break;
		default:
			throw new IllegalArgumentException("bad Element type");
		}
		return s;
	}

    /*
     * Return a string representation of the element.
     *  eg. Rz(q1), Tx(L1), Rx(90), DH(....)
     */
	public String toString() {

		String s = typeName[type] + "(";
		s += argString();
		s += ")";
		return s;
	}

    /*
    public String rotation() {
    }

    public String translation() {
    }
    */
}

/**********************************************************************
/* A list of Elements.  Subclass of Java's arrayList
 *
 *	public int factorize(int dhWhich, int verbose) 
 *
 *	public int floatRight() {
 *	public int swap(int dhWhich) {
 *	public int substituteToZ() {
 *	public int substituteToZ2() {
 *	public int substituteY() {
 *	public int merge() {
 *	public void simplify() {
 *	public ElementList() {	// constructor, use superclass
 *	public String toString() {
 */
class ElementList extends ArrayList {

    /**
     * Attempt to group this and subsequent elements into a DH term
     * @return: the number of factors matched, zero means no DH term found
     *
     * Modifies the ElementList and compresses the terms.
     */
	public int factorize(int dhWhich, int verbose) {

		int	match, jvars;
		int	i, j, f;
		Element	e;
		int	nfactors = 0;

		for (i=0; i<this.size(); i++) {
			j = i;
			jvars = match = 0;
			for (f=0; f<4; f++) {
				if (j >= this.size())
					break;
				e = (Element) this.get(j);
				if ((f == 0) && (verbose > 0))
					System.out.println("Starting at " + e);
				if (e.factorMatch(dhWhich, f, verbose)
				) {
					j++;	// move on to next element
					match++;
					if (e.isjoint())
						jvars++;
					if (jvars > 1)	// can only have 1 joint var per DH
						break;
				}
			}

			if ((match == 0) || (jvars == 0))
				continue;		// no DH subexpression found, keep looking

			int	start, end;
			if (verbose > 0)
				System.out.println(" found subexpression " + match + " " + jvars);

			start = i;
			end = j;
			if (jvars > 1)
				end--;

			Element dh = new Element(dhWhich);

			for (j=start; j<end; j++) {
				dh.add( (Element) this.get(i) );
				this.remove(i);
			}
			this.add(i, dh);
			nfactors++;
			if (verbose > 0)
					System.out.println(" result: " + dh);
		}
		return nfactors;
	}
		
	/**
	 * Attempt to 'float' translational terms as far to the right as
	 * possible and across joint boundaries.
	 */
	public int floatRight() {
		Element	e, f = null;
		int	nchanges = 0;
		int		i, j;
		boolean	crossed;

		for (i=0; i<(this.size()-1); i++) {
			e = (Element) this.get(i);
			if (e.isjoint())
				continue;
			if (!e.istrans())
				continue;
			f = null;
			crossed = false;
			for (j=i+1; j<(this.size()-1); j++) {
				f = (Element) this.get(j);
				if (f.istrans())
					continue;
				if (f.isrot() && (f.axis() == e.axis())) {
					crossed = true;
					continue;
				}
				break;
			}
			if (crossed && (f != null)) {
				System.out.println("Float: " + e + " to " + f);
				this.remove(i);
				this.add(j-1, e);
				nchanges++;
				i--;
			}
		}

		return nchanges;
	}

	/**
	 * Swap adjacent terms according to inbuilt rules so as to achieve
	 * desired term ordering.
	 */
	public int swap(int dhWhich) {
		Element	e;
		int	total_changes = 0;
		int	nchanges = 0;

		do {
				nchanges = 0;

				for (int i=0; i<(this.size()-1); i++) {
					e = (Element) this.get(i);
					if (e.swap( (Element) this.get(i+1), dhWhich)) {
						this.remove(i);
						this.add(i+1, e);
						nchanges++;
					}
				}
				total_changes += nchanges;
		} while (nchanges > 0);

		return total_changes;
	}

	/**
	 * substitute all non Z joint transforms according to rules.
	 */
	public int substituteToZ() {
		Element	e;
		Element[] replacement;
		int	nchanges = 0;

		for (int i=0; i<this.size(); i++) {
				e = (Element) this.get(i);
				if (!e.isjoint())
					continue;
				replacement = e.substituteToZ();
				if (replacement != null) {
					// diagnostic string
					System.out.print("ReplaceToZ: " + e + " := ");
					for (int j=0; j<replacement.length; j++)
						System.out.print(replacement[j]);
					System.out.println();

					this.remove(i);
					for (int j=replacement.length-1; j>=0; j--)
						this.add(i, replacement[j]);
					i += replacement.length-1;
					nchanges++;
				}
		}
		return nchanges;
	}

	/**
	 * substitute all non Z joint transforms according to rules.
	 */
	public int substituteToZ2() {
		Element	e, prev;
		Element[] replacement;
		int	nchanges = 0;
		boolean jointYet = false;

		for (int i=0; i<this.size(); i++) {
				e = (Element) this.get(i);
				if (e.isjoint())
					jointYet = true;
				if (e.isjoint())
					continue;
				if ((i == 0) || !jointYet)	// leave initial const xform
					continue;
				prev = (Element) this.get(i-1);
				//System.out.println("in ToZ2: " + e + " " + prev);
				replacement = e.substituteToZ(prev);
				if (replacement != null) {
					// diagnostic string
					System.out.print("ReplaceToZ2: " + e + " := ");
					for (int j=0; j<replacement.length; j++)
						System.out.print(replacement[j]);
					System.out.println();

					this.remove(i);
					for (int j=replacement.length-1; j>=0; j--)
						this.add(i, replacement[j]);
					i += replacement.length-1;
					nchanges++;
				}
		}
		return nchanges;
	}

	/**
	 * substitute transforms according to rules.
	 */
	public int substituteY() {
		Element	e, prev, next;
		Element[] replacement;
		int	nchanges = 0;
		boolean	jointYet = false;

		for (int i=1; i<this.size(); i++) {
				e = (Element) this.get(i);
				if (e.isjoint())
					jointYet = true;
				if ((i == 0) || !jointYet)	// leave initial const xform
					continue;
				prev = (Element) this.get(i-1);
				if ((i+1) < this.size())
					next = (Element) this.get(i+1);
				else
					next = null;
				replacement = e.substituteY(prev, next);
				if (replacement != null) {
					// diagnostic string
					System.out.print("ReplaceY: " + prev + e + " := ");
					for (int j=0; j<replacement.length; j++)
						System.out.print(replacement[j]);
					System.out.println();

					this.remove(i);
					this.remove(i-1);
					for (int j=replacement.length-1; j>=0; j--)
						this.add(i-1, replacement[j]);
					i += replacement.length-2;
					nchanges++;
				}
		}
		return nchanges;
	}

	/**
	 * merge adjacent transforms according to rules.
	 */
	public int merge() {
		int nchanges = 0;
		Element	e;

		for (int i=0; i<(this.size()-1); i++) {
			e = (Element) this.get(i);
			e = e.merge( (Element) this.get(i+1));
			if (e == this.get(i))
				continue;
			this.remove(i);
			this.remove(i);
			if (e != null)
				this.add(i, e);
			nchanges++;
		}

		return nchanges;
	}

	/**
	 * simplify expression.  Cycle continually around merging, substituting,
	 * and swapping until no more changes occur.
	 */
	public void simplify() {
		int	nchanges;
		int	nloops = 0;

		/*
		 * simplify as much as possible, then subsitute for all
		 * joint variables not in Z.
		 */
		this.merge();
		this.swap(Element.DH_STANDARD);
		this.merge();
		System.out.println(this);
		this.floatRight();
		this.merge();
		System.out.println("initial merge + swap");
		System.out.println(this);
		this.substituteToZ();
		this.merge();
		System.out.println("joint vars to Z");
		System.out.println(this);
		System.out.println("0---------------------------------------");
		do {
			nchanges = 0;

			nchanges += this.merge();
			nchanges += this.swap(Element.DH_STANDARD);
			nchanges += this.merge();
			nchanges += this.substituteY();
			nchanges += this.merge();
			System.out.println(this);
			System.out.println("1---------------------------------------");
			if (nchanges == 0) {
				System.out.println("** deal with Ry/Ty");
				nchanges += this.substituteToZ2();
				nchanges += this.merge();
			}
		} while ((nchanges > 0) && (nloops++ < 10));
	}

	public ElementList() {	// constructur, use superclass
		super();
	}

	public String toString() {
		String	s = "";

		for (int i=0; i<this.size(); i++)
			s += this.get(i) + (i < (this.size()-1) ? "." : "");

		return s;
	}

    /*
    static String jstr2offset(String s) {
        int i;
        
        if (s != null) {
            i = s.indexOf("+");
            if (i >= 0)
                return s.substring(i);
            i = s.indexOf("-");
            if (i >= 0)
                return s.substring(i);
        }
        return "0";
    }

    static String convertMatlab(String s)
    {
        if (s == null)
            return " 0";
        return " " + s;
    }

    public String toMatlab(String robot) {
		String	dh = "[";
        String  offs = "[";
        String  base = "";
        String  tool = "";
        String  theta, a, d, alpha;
        int     dhSeenYet = 0;
        Element e;

		for (int i=0; i<this.size(); i++) {
			e = (Element) this.get(i);
            if (e.type == Element.DH_STANDARD) {
                dhSeenYet = 1;
                // build up the string: theta a d alpha
                if (e.prismatic == 1) {
                    // prismatic joint
                    d = "0";    // by definition
                    offs += jstr2offset(e.D) + " ";
                    theta = (e.theta == null) ? "0" : e.theta;
                } else {
                    // revolute joint
                    theta = "0";    // by definition
                    offs += jstr2offset(e.theta) + " ";
                    d = (e.D == null) ? "0" : e.D;
                }

                a = (e.A == null) ? "0" : e.A;
                alpha = (e.alpha == null) ? "0" : e.alpha;

                s += theta;
                s += ", ";
                s += a;
                s += ", ";
                s += d;
                s += ", ";
                s += alpha;
                s += "; ";
            } else {
                // found some primitive transform, these will be
                // part of base or tool
                // scrape the leading * off
                if (xform.length() > 0)
                    xform = xform.substring(2);

                // assign this string to base or tool depending on
                // whether or not we've seen the DH terms go by
                if (dhSeenYet == 0)
                    base = xform;
                else
                    tool = xform;
            }
        }
        dh += "]";
        offs += "]";

        // build the matlab string

        s = "robot(" + dh + ", " + robot ;

        if (base.length() > 0)
            s += ", 'base', " + base;
        if (tool.length() > 0)
            s += ", 'tool', " + tool;
        s += ");";

		return s;
	}
    */
}

public class DHFactor {

    ElementList  results;

    // Matlab callable constructor
	public DHFactor(String src) {
        results = parseString(src);
        if (!this.isValid())
            System.out.println("DHFactor: error: Incomplete factorization, no DH equivalent found");
    }

    private String angle(Element e) {
        return angle(e.constant);
    }


    public String toString() {
        return results.toString();
    }

    public String display() {
        return results.toString();
    }

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
            Element e = (Element) results.get(i);

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
        Element e;

		for (int i=0; i<results.size(); i++) {
			e = (Element) results.get(i);
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
			e = (Element) results.get(i);
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
			e = (Element) results.get(i);
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
            Element e = (Element)results.get(i);

            if ( (e.type == Element.DH_STANDARD) || (e.type == Element.DH_MODIFIED) )
                return el2matlab(0, i);
        }

        return "eye(4,4)";
    }

    // return base transform string in Matlab Toolbox form
    public String tool() {
        int i;

        for (i=results.size()-1; i>=0; i--) {
            Element e = (Element)results.get(i);

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

	public static ElementList parseString(String buffer) {
		ElementList l = new ElementList();

		try {
			System.out.println(buffer);
			StringTokenizer tokens = new StringTokenizer(buffer, " *.");

			while (tokens.hasMoreTokens())
				l.add( new Element(tokens.nextToken()) );

			System.out.println(l);

			l.simplify();
			System.out.println(l);

			l.factorize(Element.DH_STANDARD, 0);
			System.out.println(l);

            return l;
		}
		catch (IllegalArgumentException e) {
			System.err.println(e.getMessage());
			System.exit(1);
		}
        return null;
	}

    // command line instantiation
    //   dhfactor file
    //   dhfactor < stdin
	public static void main(String args[]) {

        if (args.length > 0) {

            ElementList l  = parseFile(args[0]);
            System.err.println( l );
            
        } else {
            System.err.println("no file name specified\n");
            Element.showRuleUsage();
            System.exit(1);
        }
    }
}
