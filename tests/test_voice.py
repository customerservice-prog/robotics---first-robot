from app.services.voice import OfflineVoiceAssistant, extract_wake_command


def test_extract_wake_command_with_inline_command():
    assert extract_wake_command("Hey Ribitics what is on my schedule", "hey ribitics") == (
        True,
        "what is on my schedule",
    )


def test_extract_wake_command_waits_when_only_wake_phrase():
    assert extract_wake_command("Hey, Ribitics!", "hey ribitics") == (True, "")


def test_extract_wake_command_ignores_regular_speech():
    assert extract_wake_command("what is on my schedule", "hey ribitics") == (False, "")


class DummyConversation:
    async def chat(self, _message):
        raise AssertionError("should not be called")


class DummySpeaker:
    is_speaking = False

    def speak(self, _text):
        return False


def test_voice_service_starts_stopped_without_audio_dependencies():
    voice = OfflineVoiceAssistant(
        DummyConversation(),
        DummySpeaker(),
        auto_start=False,
        engine="vosk",
        wake_phrase="hey ribitics",
        model_path="models/not-installed",
        microphone_device=None,
        sample_rate=16000,
        block_size=4000,
        command_timeout_seconds=8,
    )
    status = voice.status()
    assert status.running is False
    assert status.ready is False
    assert status.state == "stopped"
