import os
import time
import numpy as np
import matplotlib.pyplot as plt


def is_power_of_two(number: int) -> bool:
    """
    Verifica si un entero positivo es potencia de 2.
    """
    return number > 0 and (number & (number - 1)) == 0


def dft(signal: np.ndarray) -> np.ndarray:
    """
    Calcula la Transformada Discreta de Fourier (DFT)
    directamente a partir de su definición matemática.
    """
    signal = np.asarray(signal, dtype=np.complex128)
    sample_count = signal.size
    spectrum = np.zeros(sample_count, dtype=np.complex128)

    for k in range(sample_count):
        accumulator = 0j
        for n in range(sample_count):
            angle = -2j * np.pi * k * n / sample_count
            accumulator += signal[n] * np.exp(angle)
        spectrum[k] = accumulator

    return spectrum


def fft(signal: np.ndarray) -> np.ndarray:
    """
    Calcula la FFT de una señal usando el algoritmo
    recursivo de Cooley-Tukey radix-2.

    Requiere que N sea potencia de 2.
    """
    signal = np.asarray(signal, dtype=np.complex128)
    sample_count = signal.size

    if sample_count == 1:
        return signal.copy()

    if not is_power_of_two(sample_count):
        raise ValueError("FFT requires the signal length to be a power of 2.")

    even_spectrum = fft(signal[::2])
    odd_spectrum = fft(signal[1::2])

    twiddle_factors = np.exp(
        -2j * np.pi * np.arange(sample_count // 2) / sample_count
    ) * odd_spectrum

    combined_spectrum = np.zeros(sample_count, dtype=np.complex128)
    combined_spectrum[: sample_count // 2] = even_spectrum + twiddle_factors
    combined_spectrum[sample_count // 2:] = even_spectrum - twiddle_factors

    return combined_spectrum


def create_test_signal(sample_rate: int, sample_count: int) -> tuple[np.ndarray, np.ndarray]:
    """
    Genera la señal de prueba compuesta por varias senoidales
    con diferentes amplitudes y fases, además de una componente DC.
    """
    time_axis = np.arange(sample_count) / sample_rate

    signal = (
        0.8
        + 1.0 * np.sin(2 * np.pi * 50 * time_axis)
        + 0.6 * np.sin(2 * np.pi * 120 * time_axis + np.pi / 4)
        + 0.4 * np.sin(2 * np.pi * 200 * time_axis - np.pi / 3)
    )

    return time_axis, signal


def get_one_sided_spectrum(
    spectrum: np.ndarray, sample_rate: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Obtiene el espectro de un solo lado para señales reales,
    junto con su magnitud normalizada y fase.
    """
    sample_count = spectrum.size
    half_count = sample_count // 2

    frequency_axis = np.arange(half_count + 1) * sample_rate / sample_count
    one_sided_spectrum = spectrum[: half_count + 1].copy()

    magnitude = np.abs(one_sided_spectrum) / sample_count
    if sample_count > 2:
        magnitude[1:-1] *= 2

    phase = np.angle(one_sided_spectrum)

    threshold = 0.01 * np.max(magnitude)
    phase[magnitude < threshold] = np.nan

    return frequency_axis, magnitude, phase


def print_dominant_components(
    spectrum: np.ndarray, sample_rate: int, top_count: int = 10
) -> None:
    """
    Imprime las componentes espectrales dominantes con su
    frecuencia, parte real, parte imaginaria, magnitud y fase.
    """
    frequency_axis, magnitude, phase = get_one_sided_spectrum(spectrum, sample_rate)
    sorted_indices = np.argsort(magnitude)[::-1]

    print("\nDominant spectral components:")
    print(
        f"{'Bin':>6} {'Freq(Hz)':>12} {'Real':>14} {'Imag':>14} "
        f"{'Magnitude':>14} {'Phase(rad)':>14}"
    )

    printed = 0
    for index in sorted_indices:
        if printed >= top_count:
            break

        complex_value = spectrum[index]
        print(
            f"{index:6d} {frequency_axis[index]:12.2f} "
            f"{complex_value.real:14.6f} {complex_value.imag:14.6f} "
            f"{magnitude[index]:14.6f} {phase[index]:14.6f}"
        )
        printed += 1


def measure_execution_time(
    transform_function, signal: np.ndarray, repetitions: int = 3
) -> tuple[float, np.ndarray]:
    """
    Mide el tiempo promedio de ejecución de una función de transformación.
    """
    elapsed_times = []
    last_result = None

    for _ in range(repetitions):
        start_time = time.perf_counter()
        last_result = transform_function(signal)
        end_time = time.perf_counter()
        elapsed_times.append(end_time - start_time)

    average_time = float(np.mean(elapsed_times))
    return average_time, last_result


def run_timing_experiment(sample_sizes: list[int], repetitions: int = 3) -> tuple[list[float], list[float]]:
    """
    Ejecuta una comparación de tiempos entre DFT y FFT
    para diferentes tamaños de señal.
    """
    dft_times = []
    fft_times = []
    sample_rate = 1024

    for sample_count in sample_sizes:
        _, signal = create_test_signal(sample_rate, sample_count)

        dft_time, _ = measure_execution_time(dft, signal, repetitions=repetitions)
        fft_time, _ = measure_execution_time(fft, signal, repetitions=repetitions)

        dft_times.append(dft_time)
        fft_times.append(fft_time)

        print(
            f"N = {sample_count:4d} | "
            f"DFT = {dft_time:.6f} s | FFT = {fft_time:.6f} s"
        )

    return dft_times, fft_times


def ensure_images_folder(folder_name: str = "images") -> str:
    """
    Crea la carpeta de salida para imágenes si no existe.
    """
    os.makedirs(folder_name, exist_ok=True)
    return folder_name


def build_image_path(folder_name: str, file_name: str) -> str:
    """
    Construye la ruta completa de una imagen dentro de la carpeta indicada.
    """
    return os.path.join(folder_name, file_name)


def plot_time_signal(time_axis: np.ndarray, signal: np.ndarray, file_path: str) -> None:
    """
    Grafica la señal en el dominio del tiempo.
    """
    plt.figure(figsize=(10, 4))
    plt.plot(time_axis, signal, linewidth=1.2)
    plt.title("Signal in the Time Domain")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(file_path, dpi=300)
    plt.show()


def plot_transform_summary(
    spectrum: np.ndarray, sample_rate: int, file_path: str, transform_name: str
) -> None:
    """
    Grafica en una sola figura la representación compleja,
    la magnitud y la fase de la transformada.
    """
    sample_count = spectrum.size
    half_count = sample_count // 2
    frequency_axis = np.arange(half_count + 1) * sample_rate / sample_count
    one_sided_spectrum = spectrum[: half_count + 1]

    _, magnitude, phase = get_one_sided_spectrum(spectrum, sample_rate)

    plt.figure(figsize=(10, 9))

    plt.subplot(3, 1, 1)
    plt.plot(frequency_axis, one_sided_spectrum.real, label="Real part")
    plt.plot(frequency_axis, one_sided_spectrum.imag, label="Imaginary part")
    plt.title(f"Complex Representation - {transform_name}")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Value")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.stem(frequency_axis, magnitude, basefmt=" ")
    plt.title(f"Magnitude Spectrum - {transform_name}")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 3)
    plt.stem(frequency_axis, phase, basefmt=" ")
    plt.title(f"Phase Spectrum - {transform_name}")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Phase (rad)")
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(file_path, dpi=300)
    plt.show()


def plot_execution_times(
    sample_sizes: list[int], dft_times: list[float], fft_times: list[float], file_path: str
) -> None:
    """
    Grafica la comparación de tiempos de ejecución entre DFT y FFT.
    """
    plt.figure(figsize=(10, 5))
    plt.plot(sample_sizes, dft_times, marker="o", label="DFT")
    plt.plot(sample_sizes, fft_times, marker="s", label="FFT")
    plt.xscale("log", base=2)
    plt.yscale("log")
    plt.title("Execution Time Comparison: DFT vs FFT")
    plt.xlabel("Signal Length N")
    plt.ylabel("Time (s)")
    plt.grid(True, which="both", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(file_path, dpi=300)
    plt.show()


def main() -> None:
    """
    Ejecuta los experimentos
    """
    images_folder = ensure_images_folder("images")

    sample_rate = 1024
    sample_count = 1024

    if not is_power_of_two(sample_count):
        raise ValueError("sample_count must be a power of 2 for FFT.")

    time_axis, signal = create_test_signal(sample_rate, sample_count)

    dft_time, dft_spectrum = measure_execution_time(dft, signal, repetitions=1)

    fft_time, fft_spectrum = measure_execution_time(fft, signal, repetitions=3)

    max_difference = np.max(np.abs(dft_spectrum - fft_spectrum))

    print("\nExperiment results:")
    print(f"DFT time  : {dft_time:.6f} s")
    print(f"FFT time  : {fft_time:.6f} s")
    print(f"Max error : {max_difference:.12e}")

    print("\nDominant components from FFT:")
    print_dominant_components(fft_spectrum, sample_rate, top_count=10)

    plot_time_signal(
        time_axis,
        signal,
        build_image_path(images_folder, "time_signal.png")
    )

    plot_transform_summary(
        dft_spectrum,
        sample_rate,
        build_image_path(images_folder, "dft_summary.png"),
        "DFT"
    )

    plot_transform_summary(
        fft_spectrum,
        sample_rate,
        build_image_path(images_folder, "fft_summary.png"),
        "FFT"
    )

    print("\nTiming experiment...")
    sample_sizes = [32, 64, 128, 256, 512, 1024]
    dft_times, fft_times = run_timing_experiment(sample_sizes, repetitions=3)

    plot_execution_times(
        sample_sizes,
        dft_times,
        fft_times,
        build_image_path(images_folder, "execution_time_comparison.png")
    )


if __name__ == "__main__":
    main()