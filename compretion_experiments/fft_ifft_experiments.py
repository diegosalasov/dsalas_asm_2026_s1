import numpy as np
import matplotlib.pyplot as plt


def is_power_of_two(number: int) -> bool:
    """
    Verifica si un entero positivo es potencia de 2.
    """
    return number > 0 and (number & (number - 1)) == 0


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


def ifft(spectrum: np.ndarray) -> np.ndarray:
    """
    Calcula la IFFT a partir de la FFT usando la relación
    de conjugación.
    """
    spectrum = np.asarray(spectrum, dtype=np.complex128)
    sample_count = spectrum.size

    if not is_power_of_two(sample_count):
        raise ValueError("IFFT requires the spectrum length to be a power of 2.")

    return np.conjugate(fft(np.conjugate(spectrum))) / sample_count


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


def build_frequency_groups(spectrum: np.ndarray) -> tuple[list[dict], float]:
    """
    Agrupa coeficientes espectrales para una señal real.

    Se conserva:
    - La componente DC por separado.
    - Cada par conjugado k y N-k como una sola componente espectral.
    - La componente de Nyquist por separado cuando N es par.
    """
    spectrum = np.asarray(spectrum, dtype=np.complex128)
    sample_count = spectrum.size
    half_count = sample_count // 2

    groups = []

    dc_energy = float(np.abs(spectrum[0]) ** 2)
    groups.append(
        {
            "indices": [0],
            "energy": dc_energy,
            "label": "DC"
        }
    )

    for k in range(1, half_count):
        pair_energy = float(
            np.abs(spectrum[k]) ** 2 + np.abs(spectrum[sample_count - k]) ** 2
        )
        groups.append(
            {
                "indices": [k, sample_count - k],
                "energy": pair_energy,
                "label": f"Pair ({k}, {sample_count - k})"
            }
        )

    nyquist_energy = float(np.abs(spectrum[half_count]) ** 2)
    groups.append(
        {
            "indices": [half_count],
            "energy": nyquist_energy,
            "label": "Nyquist"
        }
    )

    total_energy = sum(group["energy"] for group in groups)

    return groups, total_energy


def get_energy_preservation_curve(
    spectrum: np.ndarray
) -> tuple[list[dict], list[int], list[int], list[float]]:
    """
    Ordena las componentes espectrales de mayor a menor energía
    y calcula la curva de energía acumulada preservada.
    """
    groups, total_energy = build_frequency_groups(spectrum)
    sorted_groups = sorted(groups, key=lambda group: group["energy"], reverse=True)

    component_counts = []
    coefficient_counts = []
    preserved_energy_ratios = []

    cumulative_energy = 0.0
    cumulative_coefficients = 0

    for component_index, group in enumerate(sorted_groups, start=1):
        cumulative_energy += group["energy"]
        cumulative_coefficients += len(group["indices"])

        component_counts.append(component_index)
        coefficient_counts.append(cumulative_coefficients)
        preserved_energy_ratios.append(cumulative_energy / total_energy)

    return sorted_groups, component_counts, coefficient_counts, preserved_energy_ratios


def find_minimum_component_count(
    preserved_energy_ratios: list[float], target_ratio: float
) -> int:
    """
    Encuentra la cantidad mínima de componentes necesarias
    para alcanzar o superar la energía objetivo.
    """
    for index, ratio in enumerate(preserved_energy_ratios):
        if ratio >= target_ratio:
            return index + 1

    return len(preserved_energy_ratios)


def compress_spectrum(
    spectrum: np.ndarray, sorted_groups: list[dict], selected_component_count: int
) -> np.ndarray:
    """
    Construye un espectro comprimido conservando únicamente
    las componentes necesarias para la energía objetivo.
    """
    compressed_spectrum = np.zeros_like(spectrum, dtype=np.complex128)

    for group in sorted_groups[:selected_component_count]:
        for index in group["indices"]:
            compressed_spectrum[index] = spectrum[index]

    return compressed_spectrum


def reconstruct_signal(compressed_spectrum: np.ndarray) -> np.ndarray:
    """
    Reconstruye la señal en el dominio del tiempo usando IFFT.
    """
    reconstructed_signal = ifft(compressed_spectrum)

    # Se toma la parte real porque la señal original es real
    # y la parte imaginaria residual corresponde a error numérico.
    return np.real_if_close(reconstructed_signal, tol=1000).real


def compute_mse(original_signal: np.ndarray, reconstructed_signal: np.ndarray) -> float:
    """
    Calcula el error cuadrático medio entre la señal original
    y la reconstruida.
    """
    error = original_signal - reconstructed_signal
    return float(np.mean(np.abs(error) ** 2))


def compute_signal_energy(signal: np.ndarray) -> float:
    """
    Calcula la energía de una señal en el dominio del tiempo.
    """
    return float(np.sum(np.abs(signal) ** 2))


def plot_original_signal(time_axis: np.ndarray, original_signal: np.ndarray) -> None:
    """
    Grafica la señal original.
    """
    plt.figure(figsize=(10, 4))
    plt.plot(time_axis, original_signal, label="Original signal")
    plt.title("Original Signal")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_reconstructed_signal(time_axis: np.ndarray, reconstructed_signal: np.ndarray) -> None:
    """
    Grafica la señal reconstruida.
    """
    plt.figure(figsize=(10, 4))
    plt.plot(time_axis, reconstructed_signal, label="Reconstructed signal", linestyle="--")
    plt.title("Reconstructed Signal")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()


def main() -> None:
    """
    Ejecuta el experimento de compresión
    """
    sample_rate = 1024
    sample_count = 1024
    target_ratio = 0.95

    if not is_power_of_two(sample_count):
        raise ValueError("sample_count must be a power of 2 for FFT.")

    time_axis, original_signal = create_test_signal(sample_rate, sample_count)

    original_spectrum = fft(original_signal)

    sorted_groups, component_counts, coefficient_counts, preserved_energy_ratios = (
        get_energy_preservation_curve(original_spectrum)
    )

    selected_component_count = find_minimum_component_count(
        preserved_energy_ratios,
        target_ratio
    )

    selected_coefficient_count = coefficient_counts[selected_component_count - 1]
    preserved_energy = preserved_energy_ratios[selected_component_count - 1]

    compressed_spectrum = compress_spectrum(
        original_spectrum,
        sorted_groups,
        selected_component_count
    )

    reconstructed_signal = reconstruct_signal(compressed_spectrum)

    mse = compute_mse(original_signal, reconstructed_signal)

    original_energy = compute_signal_energy(original_signal)
    reconstructed_energy = compute_signal_energy(reconstructed_signal)
    preserved_energy_time = reconstructed_energy / original_energy

    print("\nCompression experiment results:")
    print(f"Target preserved energy     : {target_ratio * 100:.2f}%")
    print(f"Selected components         : {selected_component_count}")
    print(f"Kept FFT coefficients       : {selected_coefficient_count} of {sample_count}")
    print(f"Preserved energy (spectrum) : {preserved_energy * 100:.6f}%")
    print(f"Preserved energy (time)     : {preserved_energy_time * 100:.6f}%")
    print(f"MSE                         : {mse:.12e}")
    print(f"Compression ratio           : {selected_coefficient_count / sample_count:.6f}")

    plot_original_signal(time_axis, original_signal)
    plot_reconstructed_signal(time_axis, reconstructed_signal)


if __name__ == "__main__":
    main()