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
import java.util.*;

@SuppressWarnings("serial")
public class ElementList extends ArrayList<Element> {

    /**
     * Attempt to group this and subsequent elements into a DH term
     * @return: the number of factors matched, zero means no DH term found
     *
     * Modifies the ElementList and compresses the terms.
     */
	public int factorize(int dhWhich, int verbose) {

		int	match, jvars;
		int	i, j, f;
		int	nfactors = 0;

		for (i=0; i<this.size(); i++) {
			j = i;
			jvars = match = 0;
			for (f=0; f<4; f++) {
				if (j >= this.size())
					break;
                Element	e = this.get(j);
				if ((f == 0) && (verbose > 0))
					System.out.println("Starting at " + e);
                Element ee = e;
                ee.factorMatch(dhWhich, f, verbose);
                e.factorMatch(dhWhich, f, verbose);
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
				dh.add( this.get(i) );
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
			e = this.get(i);
			if (e.isjoint())
				continue;
			if (!e.istrans())
				continue;
			f = null;
			crossed = false;
			for (j=i+1; j<(this.size()-1); j++) {
				f = this.get(j);
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
		int	total_changes = 0;
		int	nchanges = 0;

		do {
				nchanges = 0;

				for (int i=0; i<(this.size()-1); i++) {
                    Element	e = this.get(i);
					if (e.swap( this.get(i+1), dhWhich)) {
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
		Element[] replacement;
		int	nchanges = 0;

		for (int i=0; i<this.size(); i++) {
                Element	e =  this.get(i);
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
				e = this.get(i);
				if (e.isjoint())
					jointYet = true;
				if (e.isjoint())
					continue;
				if ((i == 0) || !jointYet)	// leave initial const xform
					continue;
				prev = this.get(i-1);
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
		Element[] replacement;
		int	nchanges = 0;
		boolean	jointYet = false;
        Element next;

		for (int i=1; i<this.size(); i++) {
				Element e = this.get(i);
				if (e.isjoint())
					jointYet = true;
				if ((i == 0) || !jointYet)	// leave initial const xform
					continue;
				Element prev = this.get(i-1);
				if ((i+1) < this.size())
					next = this.get(i+1);
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

		for (int i=0; i<(this.size()-1); i++) {
			Element e = this.get(i);
			e = e.merge( this.get(i+1));
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

    /*
	public ElementList() {	// constructor, use superclass
		super();
	}
    */

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
