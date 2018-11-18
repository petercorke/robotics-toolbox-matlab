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
			if (s2.charAt(0) == '-')
				// handle the case where S2 begins with a negative sign
				return s1 + s2;
			else
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
        /* hack
        if (Math.abs(sum.constant) > 90)
			throw new IllegalArgumentException("rotation angle > 90");
            */

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
	 * Element constructors.  String is of the form:
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
                if (s.charAt(0) == '+')
                	s.delete(0, 1);
                
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
		String sType = s.substring(0,2);  // Tx, Rx etc
		String sRest = s.substring(2);    // the argument including brackets

		if (!(sRest.endsWith(")") && sRest.startsWith("(")))
			throw(new IllegalArgumentException("brackets"));

		for (i=0; i<6; i++)
			if (sType.equalsIgnoreCase(typeName[i]))
				break;
		if (i >= 6)
			throw(new IllegalArgumentException("bad transform name" + sType));
		type = i;


		
		sRest = sRest.substring(1, sRest.length()-1); // get the argument from between brackets
		
		// handle an optional minus sign
		String negative = "";
		

		if (sRest.charAt(0) == '-') {
			negative = "-";
			sRest = sRest.substring(1);
		}
		
		switch (sRest.charAt(0)) {
		case 'q':
			var = negative + sRest;
			break;
		case 'L':
			symconst = negative + sRest;
			break;
		default:
			try {
				constant = Integer.parseInt(sRest);
				if (negative == "-") {
					constant = -constant;
					negative = "";
				}
			}
			catch(NumberFormatException e) {
				System.err.println(e.getMessage());
				throw(new IllegalArgumentException("bad argument in term " + s));
			}
		}
		//System.out.println("ElementConstructor: " + this);
	}

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
			// Tx(L1-L3),  symconst
			// Rz(q1+180), var+constant
			// Tx(L2),     symconst
			// Ry(90),     constant
			if (var != null)
				s = var;
			if (symconst != null) {
				if (var != null)
					if (symconst.charAt(0) != '-')
						s += "+";
				s += symconst;
			}
			// constants always displayed with a sign character
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
}
