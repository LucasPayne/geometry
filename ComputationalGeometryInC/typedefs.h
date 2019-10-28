/* p18
 *   Point type definition
 */
#ifndef TYPEDEFS_DEFINED
#define TYPEDEFS_DEFINED

// See p17 on arrays versus records
#define X 0
#define Y 1

#define DIM 2
typedef int tPointi[DIM];   // Type integer point
typedef double tPointd[DIM]; // Type double point

#define PMAX 1000 /* Max # of points in a polygon */
typedef tPointi tPolygoni[PMAX]; /* Type integer polygon */

#endif
