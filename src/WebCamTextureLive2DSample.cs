﻿using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.IO;

using WebSocketSharp;
using WebSocketSharp.Net;
using WebSocketSharp.Server;

#if UNITY_5_3 || UNITY_5_3_OR_NEWER
using UnityEngine.SceneManagement;
#endif
using OpenCVForUnity;
using DlibFaceLandmarkDetector;
//using DlibFaceLandmarkDetectorSample;
using OpenCVForUnityExample;

namespace DlibFaceLandmarkDetectorWithLive2DSample
{

    public class MotionSender : WebSocketBehavior
    {

      private int counter = 0;
      private bool isConnected = false;

      protected override void OnMessage (MessageEventArgs e)
      {

      }


      protected override void OnOpen()
      {
        isConnected = true;
      }

      protected override void OnClose(CloseEventArgs e)
      {
         isConnected = false;
      }

       public void sendMsg(String msg)
        {
            counter++;
            if(counter > 5 && isConnected){
              Send(msg);
              counter = 0;
            }
        }
    }

    /// <summary>
    /// WebCamTexture sample using Dlib face landmark detection and Live2D SDK.
    /// </summary>
    [RequireComponent (typeof(WebCamTextureToMatHelper))]
    public class WebCamTextureLive2DSample : MonoBehaviour
    {

        /// <summary>
        /// The colors.
        /// </summary>
        //Color32[] colors;

        /// <summary>
        /// The texture.
        /// </summary>
        Texture2D texture;

        /// <summary>
        /// The web cam texture to mat helper.
        /// </summary>
        WebCamTextureToMatHelper webCamTextureToMatHelper = null;

        /// <summary>
        /// The face landmark detector.
        /// </summary>
        FaceLandmarkDetector faceLandmarkDetector = null;

        /// <summary>
        /// The live2DModel.
        /// </summary>
        public Live2DModel live2DModel;

        /// <summary>
        /// The frontal face parameter.
        /// </summary>
        FrontalFaceParam frontalFaceParam;

        //private List<Vector2> currentFacePoints;
        private bool isHideCameraImage = false;

        /// <summary>
        /// The shape_predictor_68_face_landmarks_dat_filepath.
        /// </summary>
        private string shape_predictor_68_face_landmarks_dat_filepath;

        private string shizuku_moc_filepath;
        private string shizuku_physics_filepath;
        private string shizuku_pose_filepath;
        private string[] texture_filepath = new string[6];

        private bool isInitedWCT2M = false;

        private WebSocketServer server = null;
        private MotionSender msender = null;

        // Use this for initialization
        void Start ()
        {
            Debug.Log("Start func called !");

            if (webCamTextureToMatHelper == null)
            {
                //webCamTextureToMatHelper = transform.root.gameObject.GetComponent<WebCamTextureToMatHelper>();
                //webCamTextureToMatHelper = GetComponent<WebCamTextureToMatHelper>();
                webCamTextureToMatHelper = gameObject.AddComponent<WebCamTextureToMatHelper>() as WebCamTextureToMatHelper;
            }

            #if UNITY_WEBGL && !UNITY_EDITOR
            webCamTextureToMatHelper.flipHorizontal = true;
            StartCoroutine(getFilePathCoroutine());
            #else
            shape_predictor_68_face_landmarks_dat_filepath = DlibFaceLandmarkDetector.Utils.getFilePath ("shape_predictor_68_face_landmarks.dat");
            shizuku_moc_filepath = OpenCVForUnity.Utils.getFilePath ("shizuku/shizuku.moc.bytes");
            shizuku_physics_filepath = OpenCVForUnity.Utils.getFilePath ("shizuku/shizuku.physics.json");
            shizuku_pose_filepath = OpenCVForUnity.Utils.getFilePath ("shizuku/shizuku.pose.json");
            for (int i = 0; i < texture_filepath.Length; i++) {
                texture_filepath [i] = OpenCVForUnity.Utils.getFilePath ("shizuku/shizuku.1024/texture_0" + i + ".png");
            }
            Run ();
            #endif

            server = new WebSocketServer(8001);

            msender = new MotionSender();
            Func<MotionSender> fn = getMotionSender;
            server.AddWebSocketService<MotionSender>("/", fn);
            server.Start();
        }

        public MotionSender getMotionSender(){
            return msender;
        }

        private IEnumerator getFilePathCoroutine ()
        {
            var getFilePathAsync_shape_predictor_68_face_landmarks_dat_filepath_Coroutine = StartCoroutine (DlibFaceLandmarkDetector.Utils.getFilePathAsync ("shape_predictor_68_face_landmarks.dat", (result) => {
                shape_predictor_68_face_landmarks_dat_filepath = result;
            }));
            var getFilePathAsync_shizuku_moc_filepath_Coroutine = StartCoroutine (OpenCVForUnity.Utils.getFilePathAsync ("shizuku/shizuku.moc.bytes", (result) => {
                shizuku_moc_filepath = result;
            }));
            var getFilePathAsync_shizuku_physics_filepath_Coroutine = StartCoroutine (OpenCVForUnity.Utils.getFilePathAsync ("shizuku/shizuku.physics.json", (result) => {
                shizuku_physics_filepath = result;
            }));
            var getFilePathAsync_shizuku_pose_filepath_Coroutine = StartCoroutine (OpenCVForUnity.Utils.getFilePathAsync ("shizuku/shizuku.pose.json", (result) => {
                shizuku_pose_filepath = result;
            }));

            yield return getFilePathAsync_shape_predictor_68_face_landmarks_dat_filepath_Coroutine;
            yield return getFilePathAsync_shizuku_moc_filepath_Coroutine;
            yield return getFilePathAsync_shizuku_physics_filepath_Coroutine;
            yield return getFilePathAsync_shizuku_pose_filepath_Coroutine;

            for (int i = 0; i < texture_filepath.Length; i++) {
                Debug.Log ("tex");
                yield return StartCoroutine (OpenCVForUnity.Utils.getFilePathAsync ("shizuku/shizuku.1024/texture_0" + i + ".png", (result) => {
                    texture_filepath [i] = result;
                }));
            }

            Run ();
        }

        private void Run ()
        {
            Debug.Log ("Run");
            if (webCamTextureToMatHelper == null)
            {
                //webCamTextureToMatHelper = gameObject.GetComponent<WebCamTextureToMatHelper>();
                //webCamTextureToMatHelper = GetComponent<WebCamTextureToMatHelper>();
                //webCamTextureToMatHelper = new WebCamTextureToMatHelper();
                webCamTextureToMatHelper = gameObject.AddComponent<WebCamTextureToMatHelper>() as WebCamTextureToMatHelper;
            }
            live2DModel.textureFiles = new Texture2D[texture_filepath.Length];
            for (int i = 0; i < texture_filepath.Length; i++)
            {
                if (string.IsNullOrEmpty(texture_filepath[i]))
                    continue;

                Texture2D tex = new Texture2D(2, 2);
                tex.LoadImage(File.ReadAllBytes(texture_filepath[i]));
                live2DModel.textureFiles[i] = tex;
            }
            if (!string.IsNullOrEmpty(shizuku_moc_filepath))
                live2DModel.setMocFileFromBytes(File.ReadAllBytes(shizuku_moc_filepath));
            if (!string.IsNullOrEmpty(shizuku_physics_filepath))
                live2DModel.setPhysicsFileFromBytes(File.ReadAllBytes(shizuku_physics_filepath));
            if (!string.IsNullOrEmpty(shizuku_pose_filepath))
                live2DModel.setPoseFileFromBytes(File.ReadAllBytes(shizuku_pose_filepath));

            Debug.Log(shape_predictor_68_face_landmarks_dat_filepath);
            faceLandmarkDetector = new FaceLandmarkDetector(shape_predictor_68_face_landmarks_dat_filepath);
            Debug.Log(faceLandmarkDetector);


            frontalFaceParam = new FrontalFaceParam ();

            webCamTextureToMatHelper.Initialize(null, webCamTextureToMatHelper.requestWidth2(), webCamTextureToMatHelper.requestHeight2(), !webCamTextureToMatHelper.requestIsFrontFacing2());
            webCamTextureToMatHelper.onInitialized.AddListener(OnWebCamTextureToMatHelperInited);
            webCamTextureToMatHelper.Initialize(null, webCamTextureToMatHelper.requestWidth2(), webCamTextureToMatHelper.requestHeight2(), !webCamTextureToMatHelper.requestIsFrontFacing2());
            isInitedWCT2M = false;
            webCamTextureToMatHelper.onInitialized.AddListener(OnWebCamTextureToMatHelperInited);

            //webCamTextureToMatHelper.Initialize ();

        }

        /// <summary>
        /// Raises the web cam texture to mat helper inited event.
        /// </summary>
        public void OnWebCamTextureToMatHelperInited ()
        {
            if (isInitedWCT2M == false)
            {
                Debug.Log("OnWebCamTextureToMatHelperInited");

                Mat webCamTextureMat = webCamTextureToMatHelper.GetMat();

                //colors = new Color32[webCamTextureMat.cols() * webCamTextureMat.rows()];
                texture = new Texture2D(webCamTextureMat.cols(), webCamTextureMat.rows(), TextureFormat.RGBA32, false);


                gameObject.transform.localScale = new Vector3(webCamTextureMat.cols(), webCamTextureMat.rows(), 1);
                Debug.Log("Screen.width " + Screen.width + " Screen.height " + Screen.height + " Screen.orientation " + Screen.orientation);

                float width = gameObject.transform.localScale.x;
                float height = gameObject.transform.localScale.y;

                float widthScale = (float)Screen.width / width;
                float heightScale = (float)Screen.height / height;
                if (widthScale < heightScale)
                {
                    Camera.main.orthographicSize = (width * (float)Screen.height / (float)Screen.width) / 2;
                }
                else
                {
                    Camera.main.orthographicSize = height / 2;
                }

                if (live2DModel != null)
                {
                    live2DModel.transform.localScale = new Vector3(Camera.main.orthographicSize, Camera.main.orthographicSize, 1);
                }

                gameObject.GetComponent<Renderer>().material.mainTexture = texture;
                //gameObject.GetComponent<Renderer>().material.mainTexture = webCamTextureToMatHelper.GetWebCamTexture();
                isInitedWCT2M = true;
            }

        }

        /// <summary>
        /// Raises the web cam texture to mat helper disposed event.
        /// </summary>
        public void OnWebCamTextureToMatHelperDisposed ()
        {
            Debug.Log ("OnWebCamTextureToMatHelperDisposed");

        }

        /// <summary>
        /// Raises the web cam texture to mat helper error occurred event.
        /// </summary>
        public void OnWebCamTextureToMatHelperErrorOccurred ()
        {
            //Debug.Log ("OnWebCamTextureToMatHelperErrorOccurred " + webCamTextureToMatHelper.GetErrorCode());
        }

        // Update is called once per frame
        void Update ()
        {
            Debug.Log("called update func");
            if (webCamTextureToMatHelper.IsPlaying () && webCamTextureToMatHelper.DidUpdateThisFrame()) {

                Mat rgbaMat = webCamTextureToMatHelper.GetMat ();
                Color32[] rgbabuf = webCamTextureToMatHelper.GetBufferColors();

                if (rgbabuf != null && faceLandmarkDetector != null && texture != null) {

                    Debug.Log("on Update above SetImage");
                    faceLandmarkDetector.SetImage<Color32> (rgbabuf, texture.width, texture.height, 4, true);

                    //detect face rects
                    List<UnityEngine.Rect> detectResult = faceLandmarkDetector.Detect ();

                    foreach (var rect in detectResult) {
                        Debug.Log ("face : " + rect);

                        //detect landmark points
                        faceLandmarkDetector.DetectLandmark (rect);

                        //draw landmark points
                        faceLandmarkDetector.DrawDetectLandmarkResult<Color32> (rgbabuf, texture.width, texture.height, 4, true, 255, 255, 255, 255);
                        //faceLandmarkDetector.DrawDetectLandmarkResult<Color32>(drawbuf, texture.width, texture.height, 4, true, 255, 255, 255, 255);

                        List<Vector2> points = faceLandmarkDetector.DetectLandmark(rect);

                        if (points.Count > 0)
                        {

                            live2DModelUpdate(points);

                        }

                    }

      
                    if(isHideCameraImage == false)
                    {
                        texture.SetPixels32(rgbabuf);
                        texture.Apply(false);
                    }

                }

            }

        }

        private void live2DModelUpdate (List<Vector2> points)
        {

            if (live2DModel != null) {

                //angle
                Vector3 angles = frontalFaceParam.getFrontalFaceAngle (points);
                float rotateX = (angles.x > 180) ? angles.x - 360 : angles.x;
                float rotateY = (angles.y > 180) ? angles.y - 360 : angles.y;
                float rotateZ = (angles.z > 180) ? angles.z - 360 : angles.z;
                live2DModel.PARAM_ANGLE.Set (-rotateY, rotateX, -rotateZ);//座標系を変換して渡す


                //eye_open_L
                float eyeOpen_L = getRaitoOfEyeOpen_L (points);
                if (eyeOpen_L > 0.8f && eyeOpen_L < 1.1f)
                    eyeOpen_L = 1;
                if (eyeOpen_L >= 1.1f)
                    eyeOpen_L = 2;
                if (eyeOpen_L < 0.7f)
                    eyeOpen_L = 0;
                live2DModel.PARAM_EYE_L_OPEN = eyeOpen_L;

                //eye_open_R
                float eyeOpen_R = getRaitoOfEyeOpen_R (points);
                if (eyeOpen_R > 0.8f && eyeOpen_R < 1.1f)
                    eyeOpen_R = 1;
                if (eyeOpen_R >= 1.1f)
                    eyeOpen_R = 2;
                if (eyeOpen_R < 0.7f)
                    eyeOpen_R = 0;
                live2DModel.PARAM_EYE_R_OPEN = eyeOpen_R;

                //eye_ball_X
                live2DModel.PARAM_EYE_BALL_X = rotateY / 60f;//視線が必ずカメラ方向を向くようにする
                //eye_ball_Y
                live2DModel.PARAM_EYE_BALL_Y = -rotateX / 60f - 0.25f;//視線が必ずカメラ方向を向くようにする

                //brow_L_Y
                float brow_L_Y = getRaitoOfBROW_L_Y (points);
                live2DModel.PARAM_BROW_L_Y = brow_L_Y;

                //brow_R_Y
                float brow_R_Y = getRaitoOfBROW_R_Y (points);
                live2DModel.PARAM_BROW_R_Y = brow_R_Y;

                //mouth_open
                float mouthOpen = getRaitoOfMouthOpen_Y (points) * 2f;
                if (mouthOpen < 0.3f)
                    mouthOpen = 0;
                if(mouthOpen > 0.6f){
                  msender.sendMsg("open");
                }else{
                  msender.sendMsg("closed");
                }
                live2DModel.PARAM_MOUTH_OPEN_Y = mouthOpen;

                //mouth_size
                float mouthSize = getRaitoOfMouthSize (points);
                live2DModel.PARAM_MOUTH_SIZE = mouthSize;

            }
        }

        //目の開き具合を算出
        private float getRaitoOfEyeOpen_L (List<Vector2> points)
        {
            if (points.Count != 68)
                throw new ArgumentNullException ("Invalid landmark_points.");

            return Mathf.Clamp (Mathf.Abs (points [43].y - points [47].y) / (Mathf.Abs (points [43].x - points [44].x) * 0.75f), -0.1f, 2.0f);
        }

        private float getRaitoOfEyeOpen_R (List<Vector2> points)
        {
            if (points.Count != 68)
                throw new ArgumentNullException ("Invalid landmark_points.");

            return Mathf.Clamp (Mathf.Abs (points [38].y - points [40].y) / (Mathf.Abs (points [37].x - points [38].x) * 0.75f), -0.1f, 2.0f);
        }

        //眉の上下
        private float getRaitoOfBROW_L_Y (List<Vector2> points)
        {
            if (points.Count != 68)
                throw new ArgumentNullException ("Invalid landmark_points.");

            float y = Mathf.Abs (points [24].y - points [27].y) / Mathf.Abs (points [27].y - points [29].y);
            y -= 1;
            y *= 4f;

            return Mathf.Clamp (y, -1.0f, 1.0f);
        }

        private float getRaitoOfBROW_R_Y (List<Vector2> points)
        {
            if (points.Count != 68)
                throw new ArgumentNullException ("Invalid landmark_points.");

            float y = Mathf.Abs (points [19].y - points [27].y) / Mathf.Abs (points [27].y - points [29].y);
            y -= 1;
            y *= 4f;

            return Mathf.Clamp (y, -1.0f, 1.0f);
        }

        //口の開き具合を算出
        private float getRaitoOfMouthOpen_Y (List<Vector2> points)
        {
            if (points.Count != 68)
                throw new ArgumentNullException ("Invalid landmark_points.");

            return Mathf.Clamp01 (Mathf.Abs (points [62].y - points [66].y) / (Mathf.Abs (points [51].y - points [62].y) + Mathf.Abs (points [66].y - points [57].y)));
        }

        //口の幅サイズを算出
        private float getRaitoOfMouthSize (List<Vector2> points)
        {
            if (points.Count != 68)
                throw new ArgumentNullException ("Invalid landmark_points.");

            float size = Mathf.Abs (points [48].x - points [54].x) / (Mathf.Abs (points [31].x - points [35].x) * 1.8f);
            size -= 1;
            size *= 4f;

            return Mathf.Clamp (size, -1.0f, 1.0f);
        }

        /// <summary>
        /// Raises the disable event.
        /// </summary>
        void OnDisable ()
        {
            if(webCamTextureToMatHelper != null) webCamTextureToMatHelper.Dispose ();

            if(faceLandmarkDetector != null) faceLandmarkDetector.Dispose ();

            if(frontalFaceParam != null) frontalFaceParam.Dispose ();
        }

        /// <summary>
        /// Raises the back button event.
        /// </summary>
        public void OnBackButton ()
        {
            #if UNITY_5_3 || UNITY_5_3_OR_NEWER
            SceneManager.LoadScene ("DlibFaceLandmarkDetectorWithLive2DSample");
            #else
            Application.LoadLevel("DlibFaceLandmarkDetectorWithLive2DSample");
            #endif
        }

        /// <summary>
        /// Raises the play button event.
        /// </summary>
        public void OnPlayButton ()
        {
            webCamTextureToMatHelper.Play ();
        }

        /// <summary>
        /// Raises the pause button event.
        /// </summary>
        public void OnPauseButton ()
        {
            webCamTextureToMatHelper.Pause ();
        }

        /// <summary>
        /// Raises the stop button event.
        /// </summary>
        public void OnStopButton ()
        {
            webCamTextureToMatHelper.Stop ();
        }

        /// <summary>
        /// Raises the change camera button event.
        /// </summary>
        public void OnChangeCameraButton ()
        {
            webCamTextureToMatHelper.Initialize (null, webCamTextureToMatHelper.requestWidth2(), webCamTextureToMatHelper.requestHeight2(), !webCamTextureToMatHelper.requestIsFrontFacing2());
        }

        /// <summary>
        /// Raises the hide camera image toggle event.
        /// </summary>
        public void OnHideCameraImageToggle ()
        {
            if (isHideCameraImage) {
                isHideCameraImage = false;
            } else {
                isHideCameraImage = true;
            }
        }
    }
}
