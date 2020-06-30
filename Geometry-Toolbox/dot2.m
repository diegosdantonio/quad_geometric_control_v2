function c = dot2(a, b, idA, idB)
%DOT2  Vector dot product.
%   This function exploits the MULTIPROD engine (MATLAB Central, file
%   #8773), which enables multiple products and array expansion (AX).
%
%   When A and B are vectors (e.g. P1, 1P, or 11P arrays):
%
%       C = DOT2(A, B) returns their scalar product. A and B must have the
%       same length. If A and B are both column vectors, DOT2(A, B) is the
%       same as CONJ(A') * B. If NDIMS(A) == 2 and NDIMS(B) == 2,
%       DOT2(A, B) is equivalent to DOT(A, B). 
%
%   More generally, when A and B are arrays of any size containing one or
%   more vectors:
%
%       C = DOT2(A, B) is equivalent to C = DOT2(A, B, IDA, IDB), where
%       IDA and IDB are the first non-singleton dimensions of A and B,
%       respectively.
%
%       C = DOT2(A, B, DIM) is equivalent to C = DOT2(A, B, IDA, IDB),
%       where IDA = IDB = DIM. If A and B have the same size, it is also
%       equivalent to C = DOT(A, B, DIM).
%
%       C = DOT2(A, B, IDA, IDB) returns the scalar products between the
%       vectors contained in A along dimension IDA and those contained in B
%       along dimension IDB. These vectors must have the same length 
%       P = SIZE(A,IDA) = SIZE(B,IDB). A and B are viewed as "block
%       arrays". IDA and IDB are referred to as their "internal dimensions"
%       (IDs). For instance, a 562 array may be viewed as an array
%       containing twelve 5-element blocks. In this case, its size is
%       denoted by (5)62, and its ID is 1. Since AX is enabled, A and B
%       may have different size, and IDA may not coincide with IDB (see
%       MULTIPROD).
%
%       C = DOT2(A, B, IDA, IDB) calls C = MULTIPROD(CONJ(A), B, IDA, IDB)
%       (MATLAB Central, file #8773), which virtually turns the vectors
%       found in CONJ(A) and B into 1P and P1 matrices, respectively,
%       then returns their products.
%
%       Input and output format (see MULTIPROD, syntax 4b):
%           Array     Block size     Internal dimension
%           -------------------------------------------
%           A         P  (1-D)       IDA
%           B         P  (1-D)       IDB
%           C         1  (1-D)       MAX(IDA, IDB)
%           -------------------------------------------
%
%   Examples:
%       If A and B are both (5)62 arrays of vectors,
%       C = DOT2(A, B) is a (1)62 array  of scalars.
%
%       A single vector B multiplies thirty vectors contained in A: 
%       If  A is .............. a 56(3) array of 30 vectors,
%       and B is .............. a (3)1   vector,
%       C = DOT2(A, B, 3, 1) is a 56(1) array of 30 scalars.
%
%   See also DOT, CROSS2, CROSSDIV, OUTER, MAGN, UNIT, PROJECTION,
%            REJECTION, MULTIPROD, TESTDOT2.

% $ Version: 2.0 $
% CODE      by:                 Paolo de Leva (IUSM, Rome, IT) 2009 Jan 24
% COMMENTS  by:                 Code author                    2009 Feb 12
% OUTPUT    tested by:          Code author                    2009 Feb 12
% -------------------------------------------------------------------------

% Allow 2 to 4 input arguments
error( nargchk(2, 4, nargin) ); 

% Setting IDA and/or IDB
switch nargin
    case 2
        idA0 = find(size(a)>1, 1, 'first'); % First non-singleton dim.
        idB0 = find(size(b)>1, 1, 'first'); % ([] if the array is a scalar)
        idA = max([idA0, 1]); % IDA = 1 if A is empty or a scalar
        idB = max([idB0, 1]);
    case 3
        idB = idA;
end

c = multiprod(conj(a), b, idA, idB);