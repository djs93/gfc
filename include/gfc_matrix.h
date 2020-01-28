#ifndef __GFC_MATRIX_H__
#define __GFC_MATRIX_H__

#include "gfc_vector.h"

typedef float Matrix4[4][4];

typedef struct mat2 {
	union {
		struct {
			float _11, _12,
				_21, _22;
		};
		float asArray[4];
	};
} mat2;

typedef struct mat3 {
	union {
		struct {
			float _11, _12, _13,
				_21, _22, _23,
				_31, _32, _33;
		};
		float asArray[9];
	};
} mat3;

void Transpose(float* srcMat, float* dstMat, int srcRows, int srcCols);
mat2 TransposeMat2(mat2 matrix);
mat3 TransposeMat3(mat3 matrix);

void Cofactor(float* out, float* minor, int rows, int cols);

mat2 CutMat3(mat3 mat, int row, int col);
mat3 Minor(mat3 mat);
mat3 CofactorMat3(mat3 mat);
float DeterminantMat3(mat3 matrix);
float DeterminantMat2(mat2 matrix);

typedef struct
{
    Matrix4 model;
    Matrix4 view;
    Matrix4 proj;
}UniformBufferObject;

/**
 * @brief copy the contents of one matrix into another
 * @param d the destination matrix
 * @param s the source matrix
 */
void gfc_matrix_copy(
    Matrix4 d,
    Matrix4 s
  );

/**
 * @brief set the matrix to an identity matrix
 * @param one the matrix to become an identity
 */
void gfc_matrix_identity(Matrix4 one);

/**
 * @brief set the matrix to a zero matrix
 * @param zero the matrix to be set to zero
 */
void gfc_matrix_zero(Matrix4 zero);

/**
 * @brief create a translation matrix given the vector
 * @param out the output matrix, the contents of this matrix are overwritten
 * @param move the vector describing the translation
 */
void gfc_matrix_make_translation(
    Matrix4 out,
    Vector3D move
);

/**
 * @brief setup a view matrix for a frustum centered at position, pointed at target, with up as the up direction
 * @note adapted from glm
 * @param out output matrix
 * @param position position of the "camera"
 * @param target location to look
 * @param up the direction considered "up"
 */
void gfc_matrix_view(
    Matrix4  out,
    Vector3D position,
    Vector3D target,
    Vector3D up
);

void gfc_matrix_slog(Matrix4 mat);

/**
 * @brief setup a perspective projection matrix
 * @note adapted from glm
 * @param out the output matrix
 * @param fov the field of view
 * @param aspect aspect ration (screen width / screen height)
 * @param near the near z plane
 * @param far the far z plane
 */
void gfc_matrix_perspective(
    Matrix4     out,
    float      fov,
    float      aspect,
    float      near,
    float      far
);


/**
 * @brief multiply the two input matrices together and save the result into out
 * @note this is not safe if out is one of the inputs
 * @param out the output matrix
 * @param a one multiplicand matrix
 * @param b another multiplicand matrix
 */
void gfc_matrix_multiply(
    Matrix4 out,
    Matrix4 a,
    Matrix4 b
  );

/**
 * @brief multiply a vector by the matrix, saving the result in an vector
 * @param out a pointer to the vector that will hold the result
 * @param mat input matrix to multiply by
 * @param vec input matrix to multiply by
 */
void gfc_matrix_multiply_vector4d(
    Vector4D * out,
    Matrix4    mat,
    Vector4D   vec
);

/**
 * @brief multiply a matrix by the rotation matrix
 * @param out the output matrix
 * @param in  the input matrix
 * @param degree the amount, in radians, to rotate by
 * @param axis the axis about which to rotate
 */
void gfc_matrix_rotate(
    Matrix4     out,
    Matrix4     in,
    float       degree,
    Vector3D    axis
);

/**
* @brief return a matrix based on the given rotation
* @param out the output matrix
* @param rotation the rotation wished to be applied to the matrix
*/
void gfc_matrix_from_rotation(Matrix4 out, Matrix4 original, Vector3D rotation);

mat2 mat2_multiply_scalar(mat2 matrix, float scalar);
mat3 mat3_multiply_scalar(mat3 matrix, float scalar);

Bool Multiply(float* out, float* matA, int aRows, int aCols, float* matB, int bRows, int bCols);
mat2 mat2_multiply(mat2 matA, mat2 matB);
mat3 mat3_multiply(mat3 matA, mat3 matB);

/**
 * @brief set the matrix to an identity matrix
 * @param one the matrix to become an identity
 */
void mat2_identity(mat2 one);

/**
 * @brief set the matrix to an identity matrix
 * @param one the matrix to become an identity
 */
void mat3_identity(mat3 one);

#endif
