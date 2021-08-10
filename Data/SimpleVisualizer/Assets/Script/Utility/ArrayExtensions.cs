using System;
using UnityEngine;

public static class ArrayExtensions {

	public static T[] Gather<T>(ref T[] array, int[] indices) {
		T[] result = new T[indices.Length];
		for(int i=0; i<indices.Length; i++) {
			result[i] = array[indices[i]];
		}
		return result;
	}

	public static T[] Gather<T>(ref T[] array, int pivot, int window) {
		int start = Mathf.Max(pivot-window, 0);
		int end = Mathf.Min(pivot+window, array.Length-1);
		T[] result = new T[end-start+1];
		for(int i=0; i<result.Length; i++) {
			result[i] = array[start+i];
		}
		return result;
	}

	public static T[] GatherByStartEnd<T>(ref T[] array, int start, int end) {
		start = Mathf.Max(start, 0);
		end = Mathf.Min(end, array.Length-1);
		T[] result = new T[end-start+1];
		for(int i=0; i<result.Length; i++) {
			result[i] = array[start+i];
		}
		return result;
	}

	public static T[] GatherByWindow<T>(ref T[] array, int start, int window) {
		start = Mathf.Max(start, 0);
		int end = Mathf.Min(start+window-1, array.Length-1);
		T[] result = new T[end-start+1];
		for(int i=0; i<result.Length; i++) {
			result[i] = array[start+i];
		}
		return result;
	}


	public static bool Same<T>(T[] a, T[] b) {
		if(a.Length != b.Length) {
			return false;
		}
		for(int i=0; i<a.Length; i++) {
			if(!a[i].Equals(b[i])) {
				return false;
			}
		}
		return true;
	}

	public static void Add<T>(ref T[] array, T element) {
		Expand(ref array);
		array[array.Length-1] = element;
	}

	public static void Insert<T>(ref T[] array, T element, int index) {
		if(index >= 0 && index < array.Length) {
			Expand(ref array);
			for(int i=array.Length-1; i>index; i--) {
				array[i] = array[i-1];
			}
			array[index] = element;
		}
	}

	public static bool RemoveAt<T>(ref T[] array, int index) {
		if(index >= 0 && index < array.Length) {
			for(int i=index; i<array.Length-1; i++) {
				array[i] = array[i+1];
			}
			Shrink(ref array);
			return true;
		} else {
			return false;
		}
	}

	public static bool Remove<T>(ref T[] array, T element) {
		return RemoveAt(ref array, FindIndex(ref array, element));
	}

	public static void Expand<T>(ref T[] array) {
		Array.Resize(ref array, array.Length+1);
	}

	public static void Shrink<T>(ref T[] array) {
		if(array.Length > 0) {
			Array.Resize(ref array, array.Length-1);
		}
	}

	public static void Resize<T>(ref T[] array, int size) {
		Array.Resize(ref array, size);
	}

	public static void Clear<T>(ref T[] array) {
		Array.Resize(ref array, 0);
	}

	public static T First<T>(this T[] array) {
		return array[0];
	}

	public static T[] First<T>(this T[,] array) {
		T[] result = new T[array.GetLength(1)];
		for(int i=0; i<result.Length; i++) {
			result[i] = array[0, i];
		}
		return result;
	}

	public static T Last<T>(this T[] array) {
		return array[array.Length-1];
	}

	public static T[] Last<T>(this T[,] array) {
		T[] result = new T[array.GetLength(1)];
		for(int i=0; i<result.Length; i++) {
			result[i] = array[array.GetLength(0)-1, i];
		}
		return result;
	}

	public static int FindIndex<T>(ref T[] array, T element) {
		return Array.FindIndex(array, x => x != null && x.Equals(element));
	}

	public static T Find<T>(ref T[] array, T element) {
		return Array.Find(array, x => x != null && x.Equals(element));
	}

	public static bool Contains<T>(ref T[] array, T element) {
		return FindIndex(ref array, element) >= 0;
	}
	public static bool Contains<T>(this T[] array, T element) {
		return FindIndex(ref array, element) >= 0;
	}

	public static T[] Concat<T>(T[] lhs, T[] rhs) {
		T[] result = new T[lhs.Length + rhs.Length];
		lhs.CopyTo(result, 0);
		rhs.CopyTo(result, lhs.Length);
		return result;
	}

	public static T[] Concat<T>(T lhs, T[] rhs) {
		return Concat(new T[1]{lhs}, rhs);
	}

	public static T[] Concat<T>(T[] lhs, T rhs) {
		return Concat(lhs, new T[1]{rhs});
	}

	public static float[] Add(float[] lhs, float[] rhs) {
		if(lhs.Length != rhs.Length) {
			Debug.Log("Incompatible array dimensions.");
			return null;
		}
		float[] result = new float[lhs.Length];
		for(int i=0; i<result.Length; i++) {
			result[i] = lhs[i] + rhs[i];
		}
		return result;
	}

	public static float[] Add(float[] lhs, float value) {
		float[] result = new float[lhs.Length];
		for(int i=0; i<result.Length; i++) {
			result[i] = lhs[i] + value;
		}
		return result;
	}

	public static float[] Sub(float[] lhs, float[] rhs) {
		if(lhs.Length != rhs.Length) {
			Debug.Log("Incompatible array dimensions.");
			return null;
		}
		float[] result = new float[lhs.Length];
		for(int i=0; i<result.Length; i++) {
			result[i] = lhs[i] - rhs[i];
		}
		return result;
	}

	public static float[] Sub(float[] lhs, float value) {
		float[] result = new float[lhs.Length];
		for(int i=0; i<result.Length; i++) {
			result[i] = lhs[i] - value;
		}
		return result;
	}

	public static int Sum(this int[] values) {
		int sum = 0;
		for(int i=0; i<values.Length; i++) {
			sum += values[i];
		}
		return sum;
	}

	public static float Sum(this float[] values) {
		float sum = 0f;
		for(int i=0; i<values.Length; i++) {
			sum += values[i];
		}
		return sum;
	}

	public static double Sum(this double[] values) {
		double sum = 0.0;
		for(int i=0; i<values.Length; i++) {
			sum += values[i];
		}
		return sum;
	}

	public static int AbsSum(this int[] values) {
		int sum = 0;
		for(int i=0; i<values.Length; i++) {
			sum += System.Math.Abs(values[i]);
		}
		return sum;
	}

	public static float AbsSum(this float[] values) {
		float sum = 0f;
		for(int i=0; i<values.Length; i++) {
			sum += System.Math.Abs(values[i]);
		}
		return sum;
	}

	public static double AbsSum(this double[] values) {
		double sum = 0.0;
		for(int i=0; i<values.Length; i++) {
			sum += System.Math.Abs(values[i]);
		}
		return sum;
	}

	public static int Min(this int[] values) {
		if(values.Length == 0) {
			return 0;
		}
		int min = int.MaxValue;
		for(int i=0; i<values.Length; i++) {
			min = Mathf.Min(min, values[i]);
		}
		return min;
	}

	public static float Min(this float[] values) {
		if(values.Length == 0) {
			return 0f;
		}
		float min = float.MaxValue;
		for(int i=0; i<values.Length; i++) {
			min = Mathf.Min(min, values[i]);
		}
		return min;
	}

	public static double Min(this double[] values) {
		if(values.Length == 0) {
			return 0.0;
		}
		double min = double.MaxValue;
		for(int i=0; i<values.Length; i++) {
			min = System.Math.Min(min, values[i]);
		}
		return min;
	}

	public static int Max(this int[] values) {
		if(values.Length == 0) {
			return 0;
		}
		int max = int.MinValue;
		for(int i=0; i<values.Length; i++) {
			max = Mathf.Max(max, values[i]);
		}
		return max;
	}

	public static float Max(this float[] values) {
		if(values.Length == 0) {
			return 0f;
		}
		float max = float.MinValue;
		for(int i=0; i<values.Length; i++) {
			max = Mathf.Max(max, values[i]);
		}
		return max;
	}

	public static double Max(this double[] values) {
		if(values.Length == 0) {
			return 0.0;
		}
		double max = double.MinValue;
		for(int i=0; i<values.Length; i++) {
			max = System.Math.Max(max, values[i]);
		}
		return max;
	}

	public static float Mean(this int[] values) {
		if(values.Length == 0) {
			return 0;
		}
		float mean = 0f;
		float args = 0f;
		for(int i=0; i<values.Length; i++) {
			mean += values[i];
			args += 1f;
		}
		mean /= args;
		return mean;
	}

	public static float Mean(this float[] values) {
		if(values.Length == 0) {
			return 0f;
		}
		float mean = 0f;
		float args = 0f;
		for(int i=0; i<values.Length; i++) {
			mean += values[i];
			args += 1f;
		}
		mean /= args;
		return mean;
	}

	public static double Mean(this double[] values) {
		if(values.Length == 0) {
			return 0.0;
		}
		double mean = 0.0;
		double args = 0.0;
		for(int i=0; i<values.Length; i++) {
			mean += values[i];
			args += 1.0;
		}
		mean /= args;
		return mean;
	}

	public static float Variance(this int[] values) {
		if(values.Length == 0) {
			return 0f;
		}
		float variance = 0f;
		float mean = values.Mean();
		float args = 0f;
		for(int i=0; i<values.Length; i++) {
			variance += Mathf.Pow(values[i] - mean, 2f);
			args += 1f;
		}
		variance /= args;
		return variance;
	}

	public static float Variance(this float[] values) {
		if(values.Length == 0) {
			return 0f;
		}
		float variance = 0f;
		float mean = values.Mean();
		float args = 0f;
		for(int i=0; i<values.Length; i++) {
			variance += Mathf.Pow(values[i] - mean, 2f);
			args += 1f;
		}
		variance /= args;
		return variance;
	}

	public static double Variance(this double[] values) {
		if(values.Length == 0) {
			return 0.0;
		}
		double variance = 0.0;
		double mean = values.Mean();
		double args = 1.0;
		for(int i=0; i<values.Length; i++) {
			variance += System.Math.Pow(values[i] - mean, 2.0);
			args += 1.0;
		}
		variance /= args;
		return variance;
	}

	public static float Sigma(this int[] values) {
		return Mathf.Sqrt(Variance(values));
	}

	public static float Sigma(this float[] values) {
		return Mathf.Sqrt(Variance(values));
	}

	public static double Sigma(this double[] values) {
		return System.Math.Sqrt(Variance(values));
	}

	public static void Zero(this float[] values) {
		for(int i=0; i<values.Length; i++) {
			values[i] = 0f;
		}
	}

	public static bool AnyTrue(this bool[] values) {
		for(int i=0; i<values.Length; i++) {
			if(values[i]) {
				return true;
			}
		}
		return false;
	}

	public static bool AnyFalse(this bool[] values) {
		for(int i=0; i<values.Length; i++) {
			if(!values[i]) {
				return true;
			}
		}
		return false;
	}

	public static int TrueCount(this bool[] values) {
		int count = 0;
		for(int i=0; i<values.Length; i++) {
			if(values[i]) {
				count += 1;
			}
		}
		return count;
	}

	public static int FalseCount(this bool[] values) {
		int count = 0;
		for(int i=0; i<values.Length; i++) {
			if(!values[i]) {
				count += 1;
			}
		}
		return count;
	}

	public static void SetAll(this bool[] array, bool value) {
		for(int i=0; i<array.Length; i++) {
			array[i] = value;
		}
	}

	public static void Print<T>(this T[] values, bool inline=false) {
		string output = string.Empty;
		for(int i=0; i<values.Length; i++) {
			output += values[i].ToString() + (inline ? " " : "\n"); 
		}
		Debug.Log(output);
	}


	// ======= bone level =======
	public static bool Verify<T>(this T[] array, int length) {
		return array != null && array.Length == length;
	}

	public static bool Any<T>(this T[] values, T value) {
		foreach(T v in values) {
			if(v.Equals(value)) {
				return true;
			}
		}
		return false;
	}

	public static T[] GatherByIndices<T>(this T[] array, params int[] indices) {
		T[] result = new T[indices.Length];
		for(int i=0; i<indices.Length; i++) {
			result[i] = array[indices[i]];
		}
		return result;
	}

	public static T[] Validate<T>(this T[] array, int length) {
		if(array == null || array.Length != length) {
			array = new T[length];
		}
		return array;
	}

	public static void SetAll<T>(this T[] array, T value) {
		for(int i=0; i<array.Length; i++) {
			array[i] = value;
		}
	}

	public static bool All<T>(this T[] values, T value) {
		foreach(T v in values) {
			if(!v.Equals(value)) {
				return false;
			}
		}
		return true;
	}

	public static int Count<T>(this T[] values, T value) {
		int count = 0;
		for(int i=0; i<values.Length; i++) {
			if(values[i].Equals(value)) {
				count += 1;
			}
		}
		return count;
	}

	public static T[] GatherByOverflowWindow<T>(this T[] array, int index, int padding) {
		int start = Mathf.Max(index-padding, 0);
		int end = Mathf.Min(index+padding, array.Length-1);
		int endOverflow = (index+padding) - end;
		int startOverflow = start - (index-padding);
		start = Mathf.Max(start-endOverflow, 0);
		end = Mathf.Min(end+startOverflow, array.Length-1);
		T[] result = new T[end-start+1];
		for(int i=0; i<result.Length; i++) {
			result[i] = array[start+i];
		}
		return result;
	}

	public static float Gaussian(this int[] values, bool[] mask, float power=1f) {
		if(values.Length == 0) {
			return 0f;
		}
		if(values.Length == 1) {
			return values[0];
		}
		float padding = ((float)values.Length - 1f) / 2f;
		float sum = 0f;
		float value = 0f;
		for(int i=0; i<values.Length; i++) {
			float weight = Mathf.Exp(-Mathf.Pow((float)i - padding, 2f) / Mathf.Pow(0.5f * padding, 2f));
			if(mask[i]) {
				if(power != 1f) {
					weight = Mathf.Pow(weight, power);
				}
				value += weight * (float)values[i];
			}
			sum += weight;
		}
		return value / sum;
	}

	public static float Gaussian(this float[] values, float power=1f) {
		if(values.Length == 0) {
			return 0f;
		}
		if(values.Length == 1) {
			return values[0];
		}
		float padding = ((float)values.Length - 1f) / 2f;
		float sum = 0f;
		float value = 0f;
		for(int i=0; i<values.Length; i++) {
			float weight = Mathf.Exp(-Mathf.Pow((float)i - padding, 2f) / Mathf.Pow(0.5f * padding, 2f));
			weight = Mathf.Pow(weight, power);
			value += weight * values[i];
			sum += weight;
		}
		return value / sum;
	}

	public static Vector3 Gaussian(this Vector3[] values, float power=1f) {
		if(values.Length == 0) {
			return Vector3.zero;
		}
		if(values.Length == 1) {
			return values[0];
		}
		float padding = ((float)values.Length - 1f) / 2f;
		float sum = 0f;
		Vector3 value = Vector3.zero;
		for(int i=0; i<values.Length; i++) {
			float weight = Mathf.Exp(-Mathf.Pow((float)i - padding, 2f) / Mathf.Pow(0.5f * padding, 2f));
			weight = Mathf.Pow(weight, power);
			value += weight * values[i];
			sum += weight;
		}
		return value / sum;
	}

	public static Matrix4x4[] GaussianFilter(this Matrix4x4[] transArray, int filterSize, float power){
		Vector3[] currentPositionArray = new Vector3[filterSize];
		Vector3[] currentForwardArray = new Vector3[filterSize];
		Vector3[] currentUpArray = new Vector3[filterSize];
		Vector3 currentPoition = Vector3.zero;
		Vector3 currentFoward = Vector3.zero;
		Vector3 currentUp = Vector3.zero;
		Quaternion currentRotation = Quaternion.identity;
		for(int i=0; i<=transArray.Length-filterSize; i++){
			for(int j=0; j<filterSize; j++){ // here could use queue
				currentPositionArray[j] = transArray[i+j].GetPosition();
				currentForwardArray[j] = transArray[i+j].GetForward();
				currentUpArray[j] = transArray[i+j].GetUp();
			}
			currentPoition = currentPositionArray.Gaussian(power);
			currentFoward = currentForwardArray.Gaussian(power);
			currentUp = currentUpArray.Gaussian(power);
			currentRotation = Quaternion.LookRotation(currentFoward, currentUp);
			transArray[i+filterSize/2] = Matrix4x4.TRS(currentPoition, currentRotation, Vector3.one);
		}
		return transArray;
	}

	public static Vector3[] GaussianFilter(this Vector3[] vectors, int filterSize, float power){
		Vector3[] currentArray = new Vector3[filterSize];
		Vector3 currentVec = Vector3.zero;
		for(int i=0; i<=vectors.Length-filterSize; i++){
			for(int j=0; j<filterSize; j++){ // here could use queue
				currentArray[j] = vectors[i+j];
			}
			currentVec = currentArray.Gaussian(power);
			vectors[i+filterSize/2] = currentVec;
		}
		return vectors;
	}

	public static float Ratio<T>(this T[] values, T value) {
		return (float)Count(values, value) / (float)values.Length;
	}
}
