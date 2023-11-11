/* testcdd1.c: Main test program to call the cdd library cddlib
   written by Komei Fukuda, fukuda@ifor.math.ethz.ch
*/

/*  This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <cddlib/setoper.h>
#include <cddlib/cdd.h>
#include <gmpxx.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <Eigen/Core>
using namespace Eigen;

dd_boolean SetInputFile(FILE **f, char *fname)
{
    dd_boolean success=dd_FALSE;

    if ( ( *f = fopen(fname, "r") )!= NULL) {
        printf("input file %s is open\n", fname);
        success=dd_TRUE;
    }
    else{
        printf("The input file %s not found\n",fname);
    }
    return success;
}

void Ab_2_MatrixPtr(const MatrixXd &A, const VectorXd &b, dd_MatrixPtr hrep, const char H_or_V) {
    // if hrep, you have Ax <= b, this function will make [A, b] become [b, -A]
    // if vrep, you have some points, this function will make b become ones(), and A
    if (A.rows() != b.rows()) throw "rows of A should be equal to rows of b";
    int dim = A.cols();

    // [A, b] to [b, -A], b - Ax >= 0
    for (int i = 0; i < A.rows(); i++) {
        for (int j = 1; j < dim + 1; j++) {
            double dataA = A(i, j - 1);
            auto s1 = hrep->matrix[i][j];
            dd_set_d(hrep->matrix[i][j], -A(i, j - 1));
            auto s = hrep->matrix[i][j];
            dd_set_d(hrep->matrix[i][j], -A(i, j - 1));
        }
    }
    for (int i = 0; i < A.rows(); i++) {
        double dataB = b(i);
        dd_set_d(hrep->matrix[i][0], dataB);
    }


    if (H_or_V == 'H') hrep->representation = dd_Inequality;
    if (H_or_V == 'V') hrep->representation = dd_Generator;
    hrep->numbtype = dd_Real;
}

int main(int argc, char *argv[])
{
    dd_PolyhedraPtr poly;
    dd_MatrixPtr M;
    dd_ErrorType err;
    dd_DataFileType inputfile;
    FILE *reading=NULL;
    dd_MatrixPtr A, G;
    dd_SetFamilyPtr GI,GA;

    MatrixXd irisA(6, 3);
    irisA << -0.99161, -0.12930,0,
            0.96328, -0.26849,0,
            1.00000,  0.00000,0,
            0.00000,  1.00000,0,
            -1.00000, 0,0,
            0, -1.00000,0;
    VectorXd irisB(6, 1);
    irisB << 2.33461, 1.91416, 5.00000, 5.00000, 5.00000, 5.00000;
//    dd_MatrixPtr hrep = dd_CreateMatrix(irisA.rows(), 2 + irisA.cols());
//    Ab_2_MatrixPtr(irisA, irisB, hrep, 'H');

    dd_set_global_constants();  /* First, this must be called. */

    dd_SetInputFile(&reading,inputfile, &err);
    if (err==dd_NoError) {
        M=dd_PolyFile2Matrix(reading, &err);
    }
    else {
        printf("Input file not found\n");
        goto _L99;
    }
// /home/kaijun/code/Third_SourceCode/cddlib-0.94m/examples-ext/mytest.ext
// /home/kaijun/code/Third_SourceCode/cddlib-0.94m/examples-ine/testmpq.ine
    if (err==dd_NoError) {
        poly=dd_DDMatrix2Poly(M, &err); /* compute the second representation */
        if (err!=dd_NoError) {
            dd_WriteErrorMessages(stdout,err);  goto _L99;
        }
        A=dd_CopyInequalities(poly);
        G=dd_CopyGenerators(poly);
        GI=dd_CopyIncidence(poly);
        GA=dd_CopyAdjacency(poly);

        if (poly->representation==dd_Inequality) {
            printf("\nInput is an H-representation\n");
        } else {
            printf("\nInput is a V-representation\n");
        }
        dd_WriteMatrix(stdout,A); printf("\n");
        dd_WriteMatrix(stdout,G);

        printf("\nHere is the incidence list:\n");
        dd_WriteSetFamily(stdout,GI);

        printf("\nHere is the adjacency list:\n");
        dd_WriteSetFamily(stdout,GA);

        dd_FreePolyhedra(poly);
        /* This is to remove all the space allocated for poly. */
        dd_FreeMatrix(M);
        dd_FreeMatrix(A);
        dd_FreeMatrix(G);
        dd_FreeSetFamily(GI);
        dd_FreeSetFamily(GA);
    } else {
        dd_WriteErrorMessages(stdout,err);
    }
    _L99:
    dd_free_global_constants();  /* At the end, this must be called. */
    return 0;
}


/* end of testcdd1.c */
