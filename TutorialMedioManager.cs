using UnityEngine;
using UnityEngine.Video;
using UnityEngine.Serialization;

public class TutorialMediaManager : MonoBehaviour
{
    // Centraliza los audios y videos de ayuda para que calibracion y juego no gestionen medios por separado.

    // --- Componentes reproductores ---
    // AudioSource reproduce voz; VideoPlayer muestra el clip asociado en la UI.
    [Header("Base Components")]
    [Tooltip("AudioSource que reproduce la voz del tutorial.")]
    public AudioSource audioSource;
    [Tooltip("VideoPlayer que muestra el video del tutorial en la UI.")]
    public VideoPlayer videoPlayer;

    // Mezcla simple: decide si un nuevo audio corta al anterior o se reproduce encima.
    [Header("Audio Mixing")]
    [Tooltip("Permite solapar clips cortos sin cortar la voz actual del tutorial.")]
    public bool allowAudioOverlap = false;
    [Range(0f, 1f)]
    public float tutorialAudioVolume = 1f;

    // Clips de voz por momento del flujo. Los FormerlySerializedAs conservan referencias antiguas en Unity.
    [Header("Audio Clips (.mp3 or .wav files)")]
    [FormerlySerializedAs("audio1_ElegirMano")]
    public AudioClip audio1_ChooseHand;
    [FormerlySerializedAs("audio2_CalibrarDerecha")]
    public AudioClip audio2_CalibrateRight;
    [FormerlySerializedAs("audio3_CalibrarCentro")]
    public AudioClip audio3_CalibrateCenter;
    [FormerlySerializedAs("audio4_CalibrarIzquierda")]
    public AudioClip audio4_CalibrateLeft;
    [FormerlySerializedAs("audio5_InicioJuego")]
    public AudioClip audio5_GameStart;
    [FormerlySerializedAs("audio6_NuevaPlataforma")]
    public AudioClip audio6_NewPlatform;

    // Videos emparejados con los audios del tutorial.
    [Header("Videos (.mp4 files)")]
    [FormerlySerializedAs("video1_ElegirMano")]
    public VideoClip video1_ChooseHand;
    [FormerlySerializedAs("video2_CalibrarDerecha")]
    public VideoClip video2_CalibrateRight;
    [FormerlySerializedAs("video3_CalibrarCentro")]
    public VideoClip video3_CalibrateCenter;
    [FormerlySerializedAs("video4_CalibrarIzquierda")]
    public VideoClip video4_CalibrateLeft;
    [Tooltip("Video usado para inicio de juego y nueva plataforma.")]
    [FormerlySerializedAs("video5_Juego")]
    public VideoClip video5_Game;

    // Indica si la voz del tutorial sigue reproduciendose.
    public bool IsAudioPlaying => audioSource != null && audioSource.isPlaying;

    // Tiempo restante del clip actual para coordinar calibracion con instrucciones.
    public float CurrentAudioRemainingTime
    {
        get
        {
            if (audioSource == null || !audioSource.isPlaying || audioSource.clip == null) return 0f;
            return Mathf.Max(0f, audioSource.clip.length - audioSource.time);
        }
    }

    private void Awake()
    {
        // Los videos se mantienen en bucle para que la pantalla no quede vacia si el usuario tarda.
        if (audioSource != null)
        {
            audioSource.playOnAwake = false;
            audioSource.loop = false;
        }

        if (videoPlayer != null)
        {
            videoPlayer.playOnAwake = false;
            videoPlayer.isLooping = true;
            videoPlayer.audioOutputMode = VideoAudioOutputMode.None;
        }
    }

    // Reproduce la indicacion inicial para elegir mano.
    public void PlayMenu() => PlayMedia(audio1_ChooseHand, video1_ChooseHand);

    // Reproduce la indicacion para calibrar el extremo derecho.
    public void PlayCalibrateRight() => PlayMedia(audio2_CalibrateRight, video2_CalibrateRight);

    // Reproduce la indicacion para calibrar el centro.
    public void PlayCalibrateCenter() => PlayMedia(audio3_CalibrateCenter, video3_CalibrateCenter);

    // Reproduce la indicacion para calibrar el extremo izquierdo.
    public void PlayCalibrateLeft() => PlayMedia(audio4_CalibrateLeft, video4_CalibrateLeft);

    // Reproduce la indicacion que prepara el inicio de la partida.
    public void PlayGameStart() => PlayMedia(audio5_GameStart, video5_Game);

    // Reproduce la indicacion que avisa de una plataforma nueva.
    public void PlayNewPlatform() => PlayMedia(audio6_NewPlatform, video5_Game);

    public void StopAllMedia()
    {
        // Se usa al terminar o reiniciar para que tutorial y juego no compitan por audio/video.
        if (audioSource != null) audioSource.Stop();
        if (videoPlayer != null) videoPlayer.Stop();
    }

    private void PlayMedia(AudioClip audio, VideoClip video)
    {
        // Audio y video cambian juntos; si el video ya es el mismo, solo se reanuda si estaba parado.
        if (audioSource != null && audio != null)
        {
            if (allowAudioOverlap)
            {
                audioSource.PlayOneShot(audio, tutorialAudioVolume);
            }
            else
            {
                audioSource.Stop();
                audioSource.clip = audio;
                audioSource.volume = tutorialAudioVolume;
                audioSource.Play();
            }
        }

        if (videoPlayer != null && video != null)
        {
            if (videoPlayer.clip != video)
            {
                videoPlayer.clip = video;
                videoPlayer.Play();
            }
            else if (!videoPlayer.isPlaying)
            {
                videoPlayer.Play();
            }
        }
    }
}
