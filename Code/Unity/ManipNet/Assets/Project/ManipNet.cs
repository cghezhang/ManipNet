using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

namespace DeepLearning {

	public class ManipNet : NativeNetwork {
		public int inputDim;

		public Encoder[] encoders = new Encoder[0];
		public DenseRes denseRes = new DenseRes();
		public bool Threading = true;

		private Matrix Xmean, Xstd, Ymean, Ystd;
		protected override void LoadDerived() {
			if(Folder==string.Empty){
				// if no folder path, try defualt one
				string rootPath = Application.dataPath;
				rootPath = rootPath.Substring(0, rootPath.Substring(0, rootPath.LastIndexOf("/")).LastIndexOf("/") + 1);
				Folder =  rootPath + "ManipNetBIN";
			}
			Xmean = CreateMatrix(inputDim, 1, "Xmean", Folder+"/Xmean.bin");
			Xstd = CreateMatrix(inputDim, 1, "Xstd", Folder+"/Xstd.bin");
			Ymean = CreateMatrix(denseRes.YDim, 1, "Ymean", Folder+"/Ymean.bin");
			Ystd = CreateMatrix(denseRes.YDim, 1, "Ystd", Folder+"/Ystd.bin");
			for(int i=0; i<encoders.Length; i++){
				encoders[i].Load(this, i);
			}
            denseRes.Load(this);

			X = CreateMatrix(inputDim, 1, "ManipNetX");
			Y = GetMatrix("DenseRes"+"Y"); // final output is also the denseRes output
		}

		 public void ReSetUp(){
			DeleteMatrices();
            LoadDerived();
            Setup = true;
            for(int i=0; i<Matrices.Count; i++) {
                if(Matrices[i] == null) {
                    Setup = false;
                    for(int j=0; j<Matrices.Count; j++) {
                        if(Matrices[j] != null) {
                            Matrices[j].Delete();
                        }
                    }
                    Matrices.Clear();
                    Debug.Log("Building network " + name + " failed.");
                }
            }
        }

		protected override void UnloadDerived() {}
		protected override void PredictDerived() {
			//Normalise Input
			Normalise(X, Xmean, Xstd, X);

            // Encode
			if(Threading) {
				List<Task> tasks = new List<Task>();
				for(int i=0; i<encoders.Length; i++) {
					int index = i;
					tasks.Add(Task.Factory.StartNew(() => encoders[index].Process(this, index)));
				}
				Task.WaitAll(tasks.ToArray());
			} else {
				for(int i=0; i<encoders.Length; i++) {
					encoders[i].Process(this, i);
				}
			}

            // DenseRes
            denseRes.Process(this);
			
			//Renormalise Output
			Renormalise(Y, Ymean, Ystd, Y);
		}

		[System.Serializable]
		public class Encoder {
			public int XDim, HDim;
			public int EncodePivot;

			private Matrix W0, b0;
			private Matrix X, Y;

			public void Load(ManipNet nn, int index) {
				if(HDim>0){
					W0 = nn.CreateMatrix(HDim, XDim, "Encoder"+index+"_w"+"0", nn.Folder+"/Encoder"+index+"_w"+"0"+".bin");

					// { // for debugging
					// 	Debug.Log("0,0" + W0.GetValue(0, 0));
					// 	Debug.Log("0,1" + W0.GetValue(0, 1));
					// 	Debug.Log("1,0" + W0.GetValue(1, 0));
					// 	Debug.Log("0,2" + W0.GetValue(0, 2));
					// 	Debug.Log("1,0" + W0.GetValue(1, 0));
					// 	Debug.Log("2,128" + W0.GetValue(2, 128));
					// 	Debug.Log("5,64" + W0.GetValue(5, 64));
					// }
					



					b0 = nn.CreateMatrix(HDim, 1, "Encoder"+index+"_b"+"0", nn.Folder+"/Encoder"+index+"_b"+"0"+".bin");
				}
				X = nn.CreateMatrix(XDim, 1, "Encoder"+index+"X");
				Y = nn.CreateMatrix(HDim, 1, "Encoder"+index+"Y");
			}

			public void Process(ManipNet nn, int index) {
				//Process Network
				if(XDim>0){
					if(HDim>0){
						for(int i=0; i<XDim; i++) {
							X.SetValue(i, 0, nn.GetInput(EncodePivot+i));
						}
						nn.Layer(X, W0, b0, Y);
					}
					else{
						for(int i=0; i<XDim; i++) {
							Y.SetValue(i, 0, nn.GetInput(EncodePivot+i));
						}
					}
				}
				else
				{
					Debug.Log("XDim of Encoder " + index + " is not given");
				}
			}
			
			public float GetLatent(int index) {
				return Y.GetValue(index, 0);
			}
		}

        [System.Serializable]
		public class DenseRes {
			public int XDim, HDim, YDim;

			private Matrix W0, W1, W2, W3, b0, b1, b2, b3, W4, b4;
			private Matrix X, Y;
            private Matrix ResBlock, ResLayer;

			public void Load(ManipNet nn) {
				if(HDim>0){
					W0 = nn.CreateMatrix(HDim, XDim, "DenseRes"+"0"+"_w0", nn.Folder+"/DenseRes"+"0"+"_w0"+".bin");
					b0 = nn.CreateMatrix(HDim, 1, "DenseRes"+"0"+"_b0", nn.Folder+"/DenseRes"+"0"+"_b0"+".bin");
					W1 = nn.CreateMatrix(HDim, HDim, "DenseRes"+"1"+"_w0", nn.Folder+"/DenseRes"+"1"+"_w0"+".bin");
					b1 = nn.CreateMatrix(HDim, 1, "DenseRes"+"1"+"_b0", nn.Folder+"/DenseRes"+"1"+"_b0"+".bin");
					W2 = nn.CreateMatrix(HDim, HDim, "DenseRes"+"2"+"_w0", nn.Folder+"/DenseRes"+"2"+"_w0"+".bin");
					b2 = nn.CreateMatrix(HDim, 1, "DenseRes"+"2"+"_b0", nn.Folder+"/DenseRes"+"2"+"_b0"+".bin");
					W3 = nn.CreateMatrix(HDim, HDim, "DenseRes"+"3"+"_w0", nn.Folder+"/DenseRes"+"3"+"_w0"+".bin");
					b3 = nn.CreateMatrix(HDim, 1, "DenseRes"+"3"+"_b0", nn.Folder+"/DenseRes"+"3"+"_b0"+".bin");

					W4 = nn.CreateMatrix(YDim, HDim, "Decoder"+"_w0", nn.Folder+"/Decoder"+"_w0"+".bin");
					b4 = nn.CreateMatrix(YDim, 1, "Decoder"+"_b0", nn.Folder+"/Decoder"+"_b0"+".bin");
				}
				X = nn.CreateMatrix(XDim, 1, "DenseRes"+"X");
				Y = nn.CreateMatrix(YDim, 1, "DenseRes"+"Y"); // this out put is also the final output
                ResBlock = nn.CreateMatrix(HDim, 1, "ResBlock");
                ResLayer = nn.CreateMatrix(HDim, 1, "ResLayer");
			}

			public void Process(ManipNet nn) {
				//Process Network
				//Set Input
				if(XDim>0 && HDim>0 && YDim>0) {
					int dim_accumulated = 0;
					for(int i=0; i<nn.encoders.Length; i++){
						for(int j=0; j<nn.encoders[i].HDim; j++){
							X.SetValue(j+dim_accumulated, 0, nn.encoders[i].GetLatent(j));
						}
						dim_accumulated += nn.encoders[i].HDim;
					}
                    //Process Network\
                    { //first block
                        X.RELU();
                        X.CopyTo(ResBlock);
                        X.CopyTo(ResLayer);
                        nn.Layer(X, W0, b0, Y);
                        Matrix.Add(Y,ResLayer,Y); 

                        Y.RELU();
                        Y.CopyTo(ResLayer);
                        nn.Layer(Y, W1, b1, Y);
                        Matrix.Add(Y,ResLayer,Y);
                        Matrix.Add(Y,ResBlock,Y);
                    }

                    { //second block
                        Y.RELU();
                        Y.CopyTo(ResBlock);
                        Y.CopyTo(ResLayer);
                        nn.Layer(Y, W2, b2, Y);
                        Matrix.Add(Y,ResLayer,Y); 

                        Y.RELU();
                        Y.CopyTo(ResLayer);
                        nn.Layer(Y, W3, b3, Y);
                        Matrix.Add(Y,ResLayer,Y);
                        Matrix.Add(Y,ResBlock,Y);
                    }

					{// decoder
						nn.Layer(Y, W4, b4, Y);
						// Y.Print();
					}
                }else{
					Debug.LogError("DenseRes Dims must be set up");
				}
			}
		}


	}

}