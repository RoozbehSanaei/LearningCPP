"""
Transformer-based image reconstruction (autoencoder) example.

This module defines a simple Vision Transformer-style encoder and
Transformer decoder that reconstructs an input image from patch tokens.
The implementation is intentionally compact and easy to adapt.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import torch
from torch import nn
from torch.utils.data import DataLoader
from torch.utils.data import random_split


@dataclass
class ViTAEConfig:
    image_size: int = 64
    patch_size: int = 8
    in_channels: int = 3
    embed_dim: int = 256
    encoder_depth: int = 6
    decoder_depth: int = 4
    num_heads: int = 8
    mlp_ratio: float = 4.0
    dropout: float = 0.1

    @property
    def num_patches(self) -> int:
        return (self.image_size // self.patch_size) ** 2


@dataclass
class AttentionIO:
    inputs: torch.Tensor
    outputs: torch.Tensor
    weights: torch.Tensor


class PatchEmbed(nn.Module):
    def __init__(self, config: ViTAEConfig) -> None:
        super().__init__()
        self.proj = nn.Conv2d(
            config.in_channels,
            config.embed_dim,
            kernel_size=config.patch_size,
            stride=config.patch_size,
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.proj(x)
        x = x.flatten(2).transpose(1, 2)
        return x


class MLP(nn.Module):
    def __init__(self, embed_dim: int, mlp_ratio: float, dropout: float) -> None:
        super().__init__()
        hidden_dim = int(embed_dim * mlp_ratio)
        self.net = nn.Sequential(
            nn.Linear(embed_dim, hidden_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, embed_dim),
            nn.Dropout(dropout),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class TransformerBlock(nn.Module):
    def __init__(self, embed_dim: int, num_heads: int, mlp_ratio: float, dropout: float) -> None:
        super().__init__()
        self.norm1 = nn.LayerNorm(embed_dim)
        self.attn = nn.MultiheadAttention(
            embed_dim=embed_dim,
            num_heads=num_heads,
            dropout=dropout,
            batch_first=True,
        )
        self.norm2 = nn.LayerNorm(embed_dim)
        self.mlp = MLP(embed_dim, mlp_ratio, dropout)

    def forward(
        self,
        x: torch.Tensor,
        return_attn: bool = False,
    ) -> torch.Tensor | tuple[torch.Tensor, AttentionIO]:
        attn_input = self.norm1(x)
        attn_output, attn_weights = self.attn(
            attn_input,
            attn_input,
            attn_input,
            need_weights=return_attn,
        )
        x = x + attn_output
        x = x + self.mlp(self.norm2(x))
        if return_attn:
            return x, AttentionIO(inputs=attn_input, outputs=attn_output, weights=attn_weights)
        return x


class ViTImageAutoencoder(nn.Module):
    def __init__(self, config: ViTAEConfig) -> None:
        super().__init__()
        self.config = config
        self.patch_embed = PatchEmbed(config)
        self.pos_embed = nn.Parameter(torch.zeros(1, config.num_patches, config.embed_dim))
        self.pos_drop = nn.Dropout(config.dropout)

        self.encoder = nn.ModuleList(
            [
                TransformerBlock(
                    config.embed_dim,
                    config.num_heads,
                    config.mlp_ratio,
                    config.dropout,
                )
                for _ in range(config.encoder_depth)
            ]
        )
        self.decoder = nn.ModuleList(
            [
                TransformerBlock(
                    config.embed_dim,
                    config.num_heads,
                    config.mlp_ratio,
                    config.dropout,
                )
                for _ in range(config.decoder_depth)
            ]
        )

        self.reconstruct = nn.Linear(
            config.embed_dim,
            config.patch_size * config.patch_size * config.in_channels,
        )

    def forward(
        self,
        x: torch.Tensor,
        return_first_attn: bool = False,
    ) -> torch.Tensor | tuple[torch.Tensor, AttentionIO | None]:
        tokens = self.patch_embed(x)
        # Positional encoding is added directly to patch tokens before attention:
        #   z = PatchEmbed(x)          [B, N, D]
        #   z = z + P                 [B, N, D]  (P = pos_embed)
        #   attn_input = LN(z)        [B, N, D]
        tokens = tokens + self.pos_embed
        tokens = self.pos_drop(tokens)
        first_attn: AttentionIO | None = None
        for idx, block in enumerate(self.encoder):
            if idx == 0 and return_first_attn:
                tokens, first_attn = block(tokens, return_attn=True)
            else:
                tokens = block(tokens)
        for block in self.decoder:
            tokens = block(tokens)
        patches = self.reconstruct(tokens)
        recon = self._unpatchify(patches)
        if return_first_attn:
            return recon, first_attn
        return recon

    def forward_with_first_attention_replacement(
        self,
        x: torch.Tensor,
        mlp_replacement: nn.Module,
    ) -> torch.Tensor:
        tokens = self.patch_embed(x)
        tokens = tokens + self.pos_embed
        tokens = self.pos_drop(tokens)
        for idx, block in enumerate(self.encoder):
            if idx == 0:
                attn_input = block.norm1(tokens)
                attn_output = mlp_replacement(attn_input)
                if attn_output.shape != attn_input.shape:
                    raise ValueError(
                        "MLP replacement must return the same shape as attention output."
                    )
                tokens = tokens + attn_output
                tokens = tokens + block.mlp(block.norm2(tokens))
            else:
                tokens = block(tokens)
        for block in self.decoder:
            tokens = block(tokens)
        patches = self.reconstruct(tokens)
        recon = self._unpatchify(patches)
        return recon

    def get_first_attention_io(self, x: torch.Tensor) -> AttentionIO:
        _, first_attn = self.forward(x, return_first_attn=True)
        if first_attn is None:
            raise RuntimeError("First attention output was not captured.")
        return first_attn

    def get_attention_io(self, x: torch.Tensor, layer_idx: int) -> AttentionIO:
        return self.get_attention_io_by_section(x, layer_idx, section="encoder")

    def get_attention_io_by_section(
        self,
        x: torch.Tensor,
        layer_idx: int,
        section: str = "encoder",
    ) -> AttentionIO:
        if section not in {"encoder", "decoder"}:
            raise ValueError("section must be 'encoder' or 'decoder'.")
        blocks = self.encoder if section == "encoder" else self.decoder
        if layer_idx < 0 or layer_idx >= len(blocks):
            raise ValueError(
                f"layer_idx must be between 0 and {len(blocks) - 1}, got {layer_idx}."
            )
        tokens = self.patch_embed(x)
        tokens = tokens + self.pos_embed
        tokens = self.pos_drop(tokens)
        if section == "encoder":
            for idx, block in enumerate(self.encoder):
                if idx == layer_idx:
                    tokens, attn = block(tokens, return_attn=True)
                    if attn is None:
                        raise RuntimeError("Attention output was not captured.")
                    return attn
                tokens = block(tokens)
        else:
            for block in self.encoder:
                tokens = block(tokens)
            for idx, block in enumerate(self.decoder):
                if idx == layer_idx:
                    tokens, attn = block(tokens, return_attn=True)
                    if attn is None:
                        raise RuntimeError("Attention output was not captured.")
                    return attn
                tokens = block(tokens)
        raise RuntimeError("Attention output was not captured.")

    def _unpatchify(self, patches: torch.Tensor) -> torch.Tensor:
        batch_size, num_patches, patch_dim = patches.shape
        patch_size = self.config.patch_size
        channels = self.config.in_channels
        height = width = int(num_patches**0.5)
        patches = patches.reshape(batch_size, height, width, patch_size, patch_size, channels)
        patches = patches.permute(0, 5, 1, 3, 2, 4)
        return patches.reshape(
            batch_size,
            channels,
            height * patch_size,
            width * patch_size,
        )

    def forward_with_attention_replacements(
        self,
        x: torch.Tensor,
        mlp_replacements: dict[int, nn.Module],
    ) -> torch.Tensor:
        tokens = self.patch_embed(x)
        tokens = tokens + self.pos_embed
        tokens = self.pos_drop(tokens)
        for idx, block in enumerate(self.encoder):
            if idx in mlp_replacements:
                attn_input = block.norm1(tokens)
                attn_output = mlp_replacements[idx](attn_input)
                if attn_output.shape != attn_input.shape:
                    raise ValueError(
                        "MLP replacement must return the same shape as attention output."
                    )
                tokens = tokens + attn_output
                tokens = tokens + block.mlp(block.norm2(tokens))
            else:
                tokens = block(tokens)
        for block in self.decoder:
            tokens = block(tokens)
        patches = self.reconstruct(tokens)
        return self._unpatchify(patches)

    def forward_with_attention_replacements_full(
        self,
        x: torch.Tensor,
        encoder_replacements: dict[int, nn.Module],
        decoder_replacements: dict[int, nn.Module],
    ) -> torch.Tensor:
        tokens = self.patch_embed(x)
        tokens = tokens + self.pos_embed
        tokens = self.pos_drop(tokens)
        for idx, block in enumerate(self.encoder):
            if idx in encoder_replacements:
                attn_input = block.norm1(tokens)
                attn_output = encoder_replacements[idx](attn_input)
                if attn_output.shape != attn_input.shape:
                    raise ValueError(
                        "MLP replacement must return the same shape as attention output."
                    )
                tokens = tokens + attn_output
                tokens = tokens + block.mlp(block.norm2(tokens))
            else:
                tokens = block(tokens)
        for idx, block in enumerate(self.decoder):
            if idx in decoder_replacements:
                attn_input = block.norm1(tokens)
                attn_output = decoder_replacements[idx](attn_input)
                if attn_output.shape != attn_input.shape:
                    raise ValueError(
                        "MLP replacement must return the same shape as attention output."
                    )
                tokens = tokens + attn_output
                tokens = tokens + block.mlp(block.norm2(tokens))
            else:
                tokens = block(tokens)
        patches = self.reconstruct(tokens)
        return self._unpatchify(patches)


def build_dataloader(
    data_dir: str,
    image_size: int,
    batch_size: int = 32,
    augment: bool = False,
) -> DataLoader:
    from torchvision import datasets, transforms

    if augment:
        transform = transforms.Compose(
            [
                transforms.RandomResizedCrop(image_size, scale=(0.8, 1.0)),
                transforms.RandomHorizontalFlip(),
                transforms.ToTensor(),
            ]
        )
    else:
        transform = transforms.Compose(
            [
                transforms.Resize((image_size, image_size)),
                transforms.ToTensor(),
            ]
        )
    dataset = datasets.ImageFolder(data_dir, transform=transform)
    return DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=2)


def build_dataloaders_with_split(
    data_dir: str,
    image_size: int,
    batch_size: int = 32,
    train_split: float = 0.8,
    seed: int | None = None,
    validate_no_overlap: bool = True,
) -> tuple[DataLoader, DataLoader]:
    from torchvision import datasets, transforms

    if not 0.0 < train_split < 1.0:
        raise ValueError("train_split must be between 0 and 1.")

    transform = transforms.Compose(
        [
            transforms.Resize((image_size, image_size)),
            transforms.ToTensor(),
        ]
    )
    dataset = datasets.ImageFolder(data_dir, transform=transform)
    train_size = int(len(dataset) * train_split)
    test_size = len(dataset) - train_size
    generator = torch.Generator().manual_seed(seed or 0)
    train_dataset, test_dataset = random_split(dataset, [train_size, test_size], generator)
    if validate_no_overlap:
        train_indices = set(train_dataset.indices)
        test_indices = set(test_dataset.indices)
        overlap = train_indices.intersection(test_indices)
        if overlap:
            raise RuntimeError(
                f"Train/test split overlap detected ({len(overlap)} shared samples)."
            )
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=2)
    test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False, num_workers=2)
    return train_loader, test_loader


def train_one_epoch(
    model: nn.Module,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    device: torch.device,
) -> float:
    model.train()
    total_loss = 0.0
    criterion = nn.MSELoss()
    for images, _ in loader:
        images = images.to(device)
        optimizer.zero_grad(set_to_none=True)
        recon = model(images)
        loss = criterion(recon, images)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * images.size(0)
    return total_loss / len(loader.dataset)


def train_one_epoch_with_accuracy(
    model: nn.Module,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    device: torch.device,
    accuracy_threshold: float,
) -> tuple[float, float]:
    model.train()
    total_loss = 0.0
    total_accuracy = 0.0
    total_samples = 0
    criterion = nn.MSELoss()
    for images, _ in loader:
        images = images.to(device)
        optimizer.zero_grad(set_to_none=True)
        recon = model(images)
        loss = criterion(recon, images)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * images.size(0)
        accuracy = (recon.sub(images).abs() < accuracy_threshold).float().mean().item()
        total_accuracy += accuracy * images.size(0)
        total_samples += images.size(0)
    avg_loss = total_loss / len(loader.dataset)
    avg_accuracy = total_accuracy / max(total_samples, 1)
    return avg_loss, avg_accuracy


def evaluate_reconstruction_accuracy_from_loader(
    model: nn.Module,
    loader: DataLoader,
    accuracy_threshold: float = 0.1,
    device: torch.device | None = None,
) -> float:
    model.eval()
    if device is None:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    total_accuracy = 0.0
    total_samples = 0
    with torch.no_grad():
        for images, _ in loader:
            images = images.to(device)
            recon = model(images)
            accuracy = (recon.sub(images).abs() < accuracy_threshold).float().mean().item()
            total_accuracy += accuracy * images.size(0)
            total_samples += images.size(0)
    return total_accuracy / max(total_samples, 1)


def run_training(
    data_dir: str,
    config: ViTAEConfig = ViTAEConfig(),
    batch_size: int = 32,
    epochs: int = 10,
    lr: float = 3e-4,
    device: str | None = None,
) -> Tuple[ViTImageAutoencoder, list[float]]:
    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    loader = build_dataloader(data_dir, config.image_size, batch_size)
    model = ViTImageAutoencoder(config).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr)

    losses = []
    for _ in range(epochs):
        epoch_loss = train_one_epoch(model, loader, optimizer, device)
        losses.append(epoch_loss)
    return model, losses


def run_training_with_accuracy(
    data_dir: str,
    config: ViTAEConfig = ViTAEConfig(),
    batch_size: int = 32,
    epochs: int = 10,
    lr: float = 3e-4,
    device: str | None = None,
    augment: bool = False,
    accuracy_threshold: float = 0.1,
) -> Tuple[ViTImageAutoencoder, list[float], list[float]]:
    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    loader = build_dataloader(data_dir, config.image_size, batch_size, augment=augment)
    model = ViTImageAutoencoder(config).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr)

    losses = []
    accuracies = []
    for _ in range(epochs):
        epoch_loss, epoch_accuracy = train_one_epoch_with_accuracy(
            model,
            loader,
            optimizer,
            device,
            accuracy_threshold=accuracy_threshold,
        )
        losses.append(epoch_loss)
        accuracies.append(epoch_accuracy)
    return model, losses, accuracies


def run_training_with_accuracy_split(
    data_dir: str,
    config: ViTAEConfig = ViTAEConfig(),
    batch_size: int = 32,
    epochs: int = 10,
    lr: float = 3e-4,
    device: str | None = None,
    accuracy_threshold: float = 0.1,
    train_split: float = 0.8,
    seed: int | None = None,
) -> Tuple[ViTImageAutoencoder, list[float], list[float]]:
    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    train_loader, test_loader = build_dataloaders_with_split(
        data_dir,
        config.image_size,
        batch_size=batch_size,
        train_split=train_split,
        seed=seed,
    )
    model = ViTImageAutoencoder(config).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr)

    losses = []
    accuracies = []
    for _ in range(epochs):
        epoch_loss = train_one_epoch(model, train_loader, optimizer, device)
        losses.append(epoch_loss)
        accuracies.append(
            evaluate_reconstruction_accuracy_from_loader(
                model,
                test_loader,
                accuracy_threshold=accuracy_threshold,
                device=device,
            )
        )
    return model, losses, accuracies


def evaluate_reconstruction_accuracy_with_mlp_replacement(
    model: ViTImageAutoencoder,
    data_dir: str,
    image_size: int,
    mlp_replacement: nn.Module,
    batch_size: int = 32,
    augment: bool = False,
    accuracy_threshold: float = 0.1,
    device: str | None = None,
) -> float:
    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    loader = build_dataloader(data_dir, image_size, batch_size, augment=augment)
    model = model.to(device)
    mlp_replacement = mlp_replacement.to(device)
    model.eval()
    mlp_replacement.eval()

    total_accuracy = 0.0
    total_samples = 0
    with torch.no_grad():
        for images, _ in loader:
            images = images.to(device)
            recon = model.forward_with_first_attention_replacement(images, mlp_replacement)
            accuracy = (recon.sub(images).abs() < accuracy_threshold).float().mean().item()
            total_accuracy += accuracy * images.size(0)
            total_samples += images.size(0)
    return total_accuracy / max(total_samples, 1)


def evaluate_reconstruction_accuracy_with_mlp_replacements(
    model: ViTImageAutoencoder,
    data_dir: str,
    image_size: int,
    mlp_replacements: dict[int, nn.Module],
    batch_size: int = 32,
    augment: bool = False,
    accuracy_threshold: float = 0.1,
    device: str | None = None,
) -> float:
    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    loader = build_dataloader(data_dir, image_size, batch_size, augment=augment)
    model = model.to(device)
    model.eval()
    for mlp in mlp_replacements.values():
        mlp.to(device)
        mlp.eval()

    total_accuracy = 0.0
    total_samples = 0
    with torch.no_grad():
        for images, _ in loader:
            images = images.to(device)
            recon = model.forward_with_attention_replacements(images, mlp_replacements)
            accuracy = (recon.sub(images).abs() < accuracy_threshold).float().mean().item()
            total_accuracy += accuracy * images.size(0)
            total_samples += images.size(0)
    return total_accuracy / max(total_samples, 1)


def evaluate_reconstruction_accuracy_with_mlp_replacements_from_loader(
    model: ViTImageAutoencoder,
    loader: DataLoader,
    mlp_replacements: dict[int, nn.Module],
    accuracy_threshold: float = 0.1,
    device: str | None = None,
) -> float:
    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model = model.to(device)
    model.eval()
    for mlp in mlp_replacements.values():
        mlp.to(device)
        mlp.eval()

    total_accuracy = 0.0
    total_samples = 0
    with torch.no_grad():
        for images, _ in loader:
            images = images.to(device)
            recon = model.forward_with_attention_replacements(images, mlp_replacements)
            accuracy = (recon.sub(images).abs() < accuracy_threshold).float().mean().item()
            total_accuracy += accuracy * images.size(0)
            total_samples += images.size(0)
    return total_accuracy / max(total_samples, 1)


def evaluate_reconstruction_accuracy_with_mlp_replacements_full_from_loader(
    model: ViTImageAutoencoder,
    loader: DataLoader,
    encoder_replacements: dict[int, nn.Module],
    decoder_replacements: dict[int, nn.Module],
    accuracy_threshold: float = 0.1,
    device: str | None = None,
) -> float:
    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model = model.to(device)
    model.eval()
    for mlp in encoder_replacements.values():
        mlp.to(device)
        mlp.eval()
    for mlp in decoder_replacements.values():
        mlp.to(device)
        mlp.eval()

    total_accuracy = 0.0
    total_samples = 0
    with torch.no_grad():
        for images, _ in loader:
            images = images.to(device)
            recon = model.forward_with_attention_replacements_full(
                images,
                encoder_replacements=encoder_replacements,
                decoder_replacements=decoder_replacements,
            )
            accuracy = (recon.sub(images).abs() < accuracy_threshold).float().mean().item()
            total_accuracy += accuracy * images.size(0)
            total_samples += images.size(0)
    return total_accuracy / max(total_samples, 1)


def save_transformer_checkpoint(model: ViTImageAutoencoder, checkpoint_path: str) -> None:
    """Save a trained transformer autoencoder checkpoint."""
    checkpoint = {
        "state_dict": model.state_dict(),
        "config": model.config,
    }
    torch.save(checkpoint, checkpoint_path)


def load_transformer_checkpoint(
    checkpoint_path: str,
    device: str | None = None,
) -> ViTImageAutoencoder:
    """Load a trained transformer autoencoder checkpoint."""
    map_location = device or ("cuda" if torch.cuda.is_available() else "cpu")
    with torch.serialization.safe_globals([ViTAEConfig]):
        checkpoint = torch.load(
            checkpoint_path,
            map_location=map_location,
            weights_only=False,
        )
    config = checkpoint["config"]
    model = ViTImageAutoencoder(config)
    model.load_state_dict(checkpoint["state_dict"])
    return model


def load_mlp_replacement_checkpoint(
    checkpoint_path: str,
    device: str | None = None,
    return_config: bool = False,
) -> SimpleMLP | tuple[SimpleMLP, ViTAEConfig | None]:
    """Load a saved attention input→output MLP replacement from disk."""
    map_location = device or ("cuda" if torch.cuda.is_available() else "cpu")
    with torch.serialization.safe_globals([ViTAEConfig]):
        checkpoint = torch.load(
            checkpoint_path,
            map_location=map_location,
            weights_only=False,
        )
    mlp = SimpleMLP(
        checkpoint["input_dim"],
        checkpoint["hidden_dim"],
        checkpoint["output_dim"],
    )
    mlp.load_state_dict(checkpoint["state_dict"])
    mlp.eval()
    if return_config:
        return mlp, checkpoint.get("config")
    return mlp


class SimpleMLP(nn.Module):
    def __init__(self, input_dim: int, hidden_dim: int, output_dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class MLPImageAutoencoder(nn.Module):
    def __init__(self, input_dim: int, hidden_dims: tuple[int, ...] = (512, 256)) -> None:
        super().__init__()
        encoder_layers: list[nn.Module] = []
        prev_dim = input_dim
        for hidden_dim in hidden_dims:
            encoder_layers.append(nn.Linear(prev_dim, hidden_dim))
            encoder_layers.append(nn.ReLU())
            prev_dim = hidden_dim
        self.encoder = nn.Sequential(*encoder_layers)

        decoder_layers: list[nn.Module] = []
        reversed_dims = list(hidden_dims)[::-1]
        for hidden_dim in reversed_dims:
            decoder_layers.append(nn.Linear(prev_dim, hidden_dim))
            decoder_layers.append(nn.ReLU())
            prev_dim = hidden_dim
        decoder_layers.append(nn.Linear(prev_dim, input_dim))
        self.decoder = nn.Sequential(*decoder_layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        batch_size = x.shape[0]
        flat = x.reshape(batch_size, -1)
        encoded = self.encoder(flat)
        decoded = self.decoder(encoded)
        return decoded.reshape_as(x)


def train_simple_mlp_on_first_attention_io(
    model: ViTImageAutoencoder,
    inputs: torch.Tensor,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    train_split: float = 0.8,
    batch_size: int = 1024,
    device: str | None = None,
    seed: int | None = None,
) -> tuple[float, float, list[float]]:
    """Train a small MLP to predict sign(attn_output) from attn_input.

    Returns (best_test_accuracy, final_test_accuracy, test_accuracy_history).
    """
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model = model.to(device)
    model.eval()
    inputs = inputs.to(device)

    with torch.no_grad():
        attn = model.get_first_attention_io(inputs)
    x = attn.inputs.reshape(-1, attn.inputs.shape[-1])
    y = (attn.outputs.reshape(-1, attn.outputs.shape[-1]) > 0).float()

    num_samples = x.shape[0]
    num_train = int(num_samples * train_split)
    perm = torch.randperm(num_samples, device=device)
    train_idx = perm[:num_train]
    test_idx = perm[num_train:]

    x_train, y_train = x[train_idx], y[train_idx]
    x_test, y_test = x[test_idx], y[test_idx]

    mlp = SimpleMLP(x.shape[-1], hidden_dim, y.shape[-1]).to(device)
    optimizer = torch.optim.Adam(mlp.parameters(), lr=lr)
    criterion = nn.BCEWithLogitsLoss()

    test_history: list[float] = []
    best_accuracy = 0.0
    for _ in range(epochs):
        mlp.train()
        perm_train = torch.randperm(x_train.shape[0], device=device)
        for start in range(0, x_train.shape[0], batch_size):
            batch_idx = perm_train[start : start + batch_size]
            optimizer.zero_grad(set_to_none=True)
            logits = mlp(x_train[batch_idx])
            loss = criterion(logits, y_train[batch_idx])
            loss.backward()
            optimizer.step()

        mlp.eval()
        with torch.no_grad():
            test_logits = mlp(x_test)
            predictions = (test_logits > 0).float()
            accuracy = (predictions == y_test).float().mean().item()
        test_history.append(accuracy)
        best_accuracy = max(best_accuracy, accuracy)

    final_accuracy = test_history[-1] if test_history else 0.0
    return best_accuracy, final_accuracy, test_history


def train_separate_mlps_on_first_attention_io(
    model: ViTImageAutoencoder,
    inputs: torch.Tensor,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    train_split: float = 0.8,
    batch_size: int = 1024,
    device: str | None = None,
    seed: int | None = None,
) -> tuple[float, float, float, float, list[float], list[float]]:
    """Train separate MLPs on attention inputs and outputs.

    Each MLP predicts the sign (>0) of its own features. Returns
    (input_best, input_final, output_best, output_final, input_history, output_history).
    """
    features, labels = _collect_attention_features(model, inputs, device=device, seed=seed)
    input_features, output_features = features
    input_labels, output_labels = labels

    return _train_separate_probes(
        input_features,
        input_labels,
        output_features,
        output_labels,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        train_split=train_split,
        batch_size=batch_size,
        device=device,
        seed=seed,
    )


def train_separate_mlps_on_first_attention_io_from_dataset(
    model: ViTImageAutoencoder,
    data_dir: str,
    image_size: int,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    train_split: float = 0.8,
    batch_size: int = 1024,
    loader_batch_size: int = 32,
    augment: bool = False,
    device: str | None = None,
    seed: int | None = None,
) -> tuple[float, float, float, float, list[float], list[float]]:
    """Train separate MLPs on attention inputs/outputs gathered from a real dataset."""
    loader = build_dataloader(
        data_dir,
        image_size,
        batch_size=loader_batch_size,
        augment=augment,
    )
    features, labels = _collect_attention_features_from_loader(
        model,
        loader,
        device=device,
        seed=seed,
    )
    input_features, output_features = features
    input_labels, output_labels = labels

    return _train_separate_probes(
        input_features,
        input_labels,
        output_features,
        output_labels,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        train_split=train_split,
        batch_size=batch_size,
        device=device,
        seed=seed,
    )


def _collect_attention_features(
    model: ViTImageAutoencoder,
    inputs: torch.Tensor,
    device: str | None = None,
    seed: int | None = None,
) -> tuple[tuple[torch.Tensor, torch.Tensor], tuple[torch.Tensor, torch.Tensor]]:
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model = model.to(device)
    model.eval()
    inputs = inputs.to(device)

    with torch.no_grad():
        attn = model.get_first_attention_io(inputs)

    input_features = attn.inputs.reshape(-1, attn.inputs.shape[-1])
    output_features = attn.outputs.reshape(-1, attn.outputs.shape[-1])
    input_labels = (input_features > 0).float()
    output_labels = (output_features > 0).float()
    return (input_features, output_features), (input_labels, output_labels)


def _collect_attention_features_raw(
    model: ViTImageAutoencoder,
    inputs: torch.Tensor,
    device: str | None = None,
    seed: int | None = None,
    layer_idx: int = 0,
    section: str = "encoder",
) -> tuple[torch.Tensor, torch.Tensor]:
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model = model.to(device)
    model.eval()
    inputs = inputs.to(device)

    with torch.no_grad():
        attn = model.get_attention_io_by_section(inputs, layer_idx, section=section)

    input_features = attn.inputs.reshape(-1, attn.inputs.shape[-1])
    output_features = attn.outputs.reshape(-1, attn.outputs.shape[-1])
    return input_features, output_features


def _collect_attention_features_from_loader(
    model: ViTImageAutoencoder,
    loader: DataLoader,
    device: str | None = None,
    seed: int | None = None,
) -> tuple[tuple[torch.Tensor, torch.Tensor], tuple[torch.Tensor, torch.Tensor]]:
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model = model.to(device)
    model.eval()

    input_batches: list[torch.Tensor] = []
    output_batches: list[torch.Tensor] = []
    with torch.no_grad():
        for images, _ in loader:
            images = images.to(device)
            attn = model.get_first_attention_io(images)
            input_batches.append(attn.inputs.reshape(-1, attn.inputs.shape[-1]))
            output_batches.append(attn.outputs.reshape(-1, attn.outputs.shape[-1]))

    input_features = torch.cat(input_batches, dim=0)
    output_features = torch.cat(output_batches, dim=0)
    input_labels = (input_features > 0).float()
    output_labels = (output_features > 0).float()
    return (input_features, output_features), (input_labels, output_labels)


def _collect_attention_features_raw_from_loader(
    model: ViTImageAutoencoder,
    loader: DataLoader,
    device: str | None = None,
    seed: int | None = None,
    layer_idx: int = 0,
    section: str = "encoder",
) -> tuple[torch.Tensor, torch.Tensor]:
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model = model.to(device)
    model.eval()

    input_batches: list[torch.Tensor] = []
    output_batches: list[torch.Tensor] = []
    with torch.no_grad():
        for images, _ in loader:
            images = images.to(device)
            attn = model.get_attention_io_by_section(images, layer_idx, section=section)
            input_batches.append(attn.inputs.reshape(-1, attn.inputs.shape[-1]))
            output_batches.append(attn.outputs.reshape(-1, attn.outputs.shape[-1]))

    input_features = torch.cat(input_batches, dim=0)
    output_features = torch.cat(output_batches, dim=0)
    return input_features, output_features


def _train_separate_probes(
    input_features: torch.Tensor,
    input_labels: torch.Tensor,
    output_features: torch.Tensor,
    output_labels: torch.Tensor,
    epochs: int,
    lr: float,
    hidden_dim: int,
    train_split: float,
    batch_size: int,
    device: str | None,
    seed: int | None,
) -> tuple[float, float, float, float, list[float], list[float]]:
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))

    def train_probe(
        features: torch.Tensor,
        labels: torch.Tensor,
    ) -> tuple[float, float, list[float]]:
        num_samples = features.shape[0]
        num_train = int(num_samples * train_split)
        perm = torch.randperm(num_samples, device=device)
        train_idx = perm[:num_train]
        test_idx = perm[num_train:]

        x_train, y_train = features[train_idx], labels[train_idx]
        x_test, y_test = features[test_idx], labels[test_idx]

        mlp = SimpleMLP(features.shape[-1], hidden_dim, labels.shape[-1]).to(device)
        optimizer = torch.optim.Adam(mlp.parameters(), lr=lr)
        criterion = nn.BCEWithLogitsLoss()

        test_history: list[float] = []
        best_accuracy = 0.0
        for _ in range(epochs):
            mlp.train()
            perm_train = torch.randperm(x_train.shape[0], device=device)
            for start in range(0, x_train.shape[0], batch_size):
                batch_idx = perm_train[start : start + batch_size]
                optimizer.zero_grad(set_to_none=True)
                logits = mlp(x_train[batch_idx])
                loss = criterion(logits, y_train[batch_idx])
                loss.backward()
                optimizer.step()

            mlp.eval()
            with torch.no_grad():
                test_logits = mlp(x_test)
                predictions = (test_logits > 0).float()
                accuracy = (predictions == y_test).float().mean().item()
            test_history.append(accuracy)
            best_accuracy = max(best_accuracy, accuracy)

        final_accuracy = test_history[-1] if test_history else 0.0
        return best_accuracy, final_accuracy, test_history

    input_best, input_final, input_history = train_probe(input_features, input_labels)
    output_best, output_final, output_history = train_probe(output_features, output_labels)
    return input_best, input_final, output_best, output_final, input_history, output_history


def train_mlp_predict_attn_inputs_from_outputs(
    model: ViTImageAutoencoder,
    inputs: torch.Tensor,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    train_split: float = 0.8,
    batch_size: int = 1024,
    device: str | None = None,
    seed: int | None = None,
) -> tuple[float, float, list[float]]:
    """Train an MLP to regress attention inputs from attention outputs.

    Returns (best_test_mse, final_test_mse, test_mse_history).
    """
    input_features, output_features = _collect_attention_features_raw(
        model,
        inputs,
        device=device,
        seed=seed,
    )
    return _train_regression_probe(
        output_features,
        input_features,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        train_split=train_split,
        batch_size=batch_size,
        device=device,
        seed=seed,
    )


def train_mlp_predict_attn_inputs_from_outputs_from_dataset(
    model: ViTImageAutoencoder,
    data_dir: str,
    image_size: int,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    train_split: float = 0.8,
    batch_size: int = 1024,
    loader_batch_size: int = 32,
    augment: bool = False,
    device: str | None = None,
    seed: int | None = None,
) -> tuple[float, float, list[float]]:
    """Train an MLP to regress attention inputs from outputs using a dataset."""
    loader = build_dataloader(
        data_dir,
        image_size,
        batch_size=loader_batch_size,
        augment=augment,
    )
    input_features, output_features = _collect_attention_features_raw_from_loader(
        model,
        loader,
        device=device,
        seed=seed,
    )
    return _train_regression_probe(
        output_features,
        input_features,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        train_split=train_split,
        batch_size=batch_size,
        device=device,
        seed=seed,
    )


def train_mlp_predict_attn_outputs_from_inputs(
    model: ViTImageAutoencoder,
    inputs: torch.Tensor,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    train_split: float = 0.8,
    batch_size: int = 1024,
    device: str | None = None,
    seed: int | None = None,
    layer_idx: int = 0,
    section: str = "encoder",
) -> tuple[SimpleMLP, float, float, list[float]]:
    """Train an MLP to regress attention outputs from attention inputs.

    Returns (mlp, best_test_mse, final_test_mse, test_mse_history).
    """
    input_features, output_features = _collect_attention_features_raw(
        model,
        inputs,
        device=device,
        seed=seed,
        layer_idx=layer_idx,
        section=section,
    )
    return _train_regression_probe_with_model(
        input_features,
        output_features,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        train_split=train_split,
        batch_size=batch_size,
        device=device,
        seed=seed,
    )


def train_mlp_predict_attn_outputs_from_inputs_from_dataset(
    model: ViTImageAutoencoder,
    data_dir: str,
    image_size: int,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    train_split: float = 0.8,
    batch_size: int = 1024,
    loader_batch_size: int = 32,
    augment: bool = False,
    device: str | None = None,
    seed: int | None = None,
    layer_idx: int = 0,
    section: str = "encoder",
) -> tuple[SimpleMLP, float, float, list[float]]:
    """Train an MLP to regress attention outputs from inputs using a dataset."""
    loader = build_dataloader(
        data_dir,
        image_size,
        batch_size=loader_batch_size,
        augment=augment,
    )
    input_features, output_features = _collect_attention_features_raw_from_loader(
        model,
        loader,
        device=device,
        seed=seed,
        layer_idx=layer_idx,
        section=section,
    )
    return _train_regression_probe_with_model(
        input_features,
        output_features,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        train_split=train_split,
        batch_size=batch_size,
        device=device,
        seed=seed,
    )


def train_mlp_predict_attn_outputs_from_inputs_for_all_layers_from_dataset(
    model: ViTImageAutoencoder,
    data_dir: str,
    image_size: int,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    train_split: float = 0.8,
    batch_size: int = 1024,
    loader_batch_size: int = 32,
    augment: bool = False,
    device: str | None = None,
    seed: int | None = None,
) -> dict[int, tuple[SimpleMLP, float, float, list[float]]]:
    """Train attention-output regression MLPs for every encoder layer."""
    results: dict[int, tuple[SimpleMLP, float, float, list[float]]] = {}
    for layer_idx in range(len(model.encoder)):
        results[layer_idx] = train_mlp_predict_attn_outputs_from_inputs_from_dataset(
            model,
            data_dir=data_dir,
            image_size=image_size,
            epochs=epochs,
            lr=lr,
            hidden_dim=hidden_dim,
            train_split=train_split,
            batch_size=batch_size,
            loader_batch_size=loader_batch_size,
            augment=augment,
            device=device,
            seed=seed,
            layer_idx=layer_idx,
        )
    return results


def train_mlp_predict_attn_outputs_from_inputs_for_all_layers_from_split_dataset(
    model: ViTImageAutoencoder,
    data_dir: str,
    image_size: int,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    batch_size: int = 1024,
    loader_batch_size: int = 32,
    train_split: float = 0.8,
    seed: int | None = None,
    device: str | None = None,
) -> tuple[dict[int, tuple[SimpleMLP, float, float, list[float]]], DataLoader]:
    """Train per-layer regression MLPs using a fixed train/test split (no augmentation)."""
    train_loader, test_loader = build_dataloaders_with_split(
        data_dir,
        image_size,
        batch_size=loader_batch_size,
        train_split=train_split,
        seed=seed,
    )
    results: dict[int, tuple[SimpleMLP, float, float, list[float]]] = {}
    for layer_idx in range(len(model.encoder)):
        train_features, train_targets = _collect_attention_features_raw_from_loader(
            model,
            train_loader,
            device=device,
            seed=seed,
            layer_idx=layer_idx,
        )
        test_features, test_targets = _collect_attention_features_raw_from_loader(
            model,
            test_loader,
            device=device,
            seed=seed,
            layer_idx=layer_idx,
        )
        results[layer_idx] = _train_regression_probe_with_model_from_splits(
            train_features=train_features,
            train_targets=train_targets,
            test_features=test_features,
            test_targets=test_targets,
            epochs=epochs,
            lr=lr,
            hidden_dim=hidden_dim,
            batch_size=batch_size,
            device=device,
            seed=seed,
        )
    return results, test_loader


def train_mlp_predict_attn_outputs_from_inputs_for_all_layers_from_split_dataset_full(
    model: ViTImageAutoencoder,
    data_dir: str,
    image_size: int,
    epochs: int = 50,
    lr: float = 1e-3,
    hidden_dim: int = 128,
    batch_size: int = 1024,
    loader_batch_size: int = 32,
    train_split: float = 0.8,
    seed: int | None = None,
    device: str | None = None,
) -> tuple[
    dict[int, tuple[SimpleMLP, float, float, list[float]]],
    dict[int, tuple[SimpleMLP, float, float, list[float]]],
    DataLoader,
]:
    """Train per-layer regression MLPs for encoder + decoder using a fixed split."""
    train_loader, test_loader = build_dataloaders_with_split(
        data_dir,
        image_size,
        batch_size=loader_batch_size,
        train_split=train_split,
        seed=seed,
    )
    encoder_results: dict[int, tuple[SimpleMLP, float, float, list[float]]] = {}
    decoder_results: dict[int, tuple[SimpleMLP, float, float, list[float]]] = {}
    for layer_idx in range(len(model.encoder)):
        train_features, train_targets = _collect_attention_features_raw_from_loader(
            model,
            train_loader,
            device=device,
            seed=seed,
            layer_idx=layer_idx,
            section="encoder",
        )
        test_features, test_targets = _collect_attention_features_raw_from_loader(
            model,
            test_loader,
            device=device,
            seed=seed,
            layer_idx=layer_idx,
            section="encoder",
        )
        encoder_results[layer_idx] = _train_regression_probe_with_model_from_splits(
            train_features=train_features,
            train_targets=train_targets,
            test_features=test_features,
            test_targets=test_targets,
            epochs=epochs,
            lr=lr,
            hidden_dim=hidden_dim,
            batch_size=batch_size,
            device=device,
            seed=seed,
        )
    for layer_idx in range(len(model.decoder)):
        train_features, train_targets = _collect_attention_features_raw_from_loader(
            model,
            train_loader,
            device=device,
            seed=seed,
            layer_idx=layer_idx,
            section="decoder",
        )
        test_features, test_targets = _collect_attention_features_raw_from_loader(
            model,
            test_loader,
            device=device,
            seed=seed,
            layer_idx=layer_idx,
            section="decoder",
        )
        decoder_results[layer_idx] = _train_regression_probe_with_model_from_splits(
            train_features=train_features,
            train_targets=train_targets,
            test_features=test_features,
            test_targets=test_targets,
            epochs=epochs,
            lr=lr,
            hidden_dim=hidden_dim,
            batch_size=batch_size,
            device=device,
            seed=seed,
        )
    return encoder_results, decoder_results, test_loader


def _train_regression_probe(
    features: torch.Tensor,
    targets: torch.Tensor,
    epochs: int,
    lr: float,
    hidden_dim: int,
    train_split: float,
    batch_size: int,
    device: str | None,
    seed: int | None,
) -> tuple[float, float, list[float]]:
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))

    num_samples = features.shape[0]
    num_train = int(num_samples * train_split)
    perm = torch.randperm(num_samples, device=device)
    train_idx = perm[:num_train]
    test_idx = perm[num_train:]

    x_train, y_train = features[train_idx], targets[train_idx]
    x_test, y_test = features[test_idx], targets[test_idx]

    mlp = SimpleMLP(features.shape[-1], hidden_dim, targets.shape[-1]).to(device)
    optimizer = torch.optim.Adam(mlp.parameters(), lr=lr)
    criterion = nn.MSELoss()

    test_history: list[float] = []
    best_mse = float("inf")
    for _ in range(epochs):
        mlp.train()
        perm_train = torch.randperm(x_train.shape[0], device=device)
        for start in range(0, x_train.shape[0], batch_size):
            batch_idx = perm_train[start : start + batch_size]
            optimizer.zero_grad(set_to_none=True)
            preds = mlp(x_train[batch_idx])
            loss = criterion(preds, y_train[batch_idx])
            loss.backward()
            optimizer.step()

        mlp.eval()
        with torch.no_grad():
            preds = mlp(x_test)
            mse = criterion(preds, y_test).item()
        test_history.append(mse)
        best_mse = min(best_mse, mse)

    final_mse = test_history[-1] if test_history else float("inf")
    return best_mse, final_mse, test_history


def _train_regression_probe_with_model(
    features: torch.Tensor,
    targets: torch.Tensor,
    epochs: int,
    lr: float,
    hidden_dim: int,
    train_split: float,
    batch_size: int,
    device: str | None,
    seed: int | None,
) -> tuple[SimpleMLP, float, float, list[float]]:
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))

    num_samples = features.shape[0]
    num_train = int(num_samples * train_split)
    perm = torch.randperm(num_samples, device=device)
    train_idx = perm[:num_train]
    test_idx = perm[num_train:]

    x_train, y_train = features[train_idx], targets[train_idx]
    x_test, y_test = features[test_idx], targets[test_idx]

    mlp = SimpleMLP(features.shape[-1], hidden_dim, targets.shape[-1]).to(device)
    optimizer = torch.optim.Adam(mlp.parameters(), lr=lr)
    criterion = nn.MSELoss()

    test_history: list[float] = []
    best_mse = float("inf")
    for _ in range(epochs):
        mlp.train()
        perm_train = torch.randperm(x_train.shape[0], device=device)
        for start in range(0, x_train.shape[0], batch_size):
            batch_idx = perm_train[start : start + batch_size]
            optimizer.zero_grad(set_to_none=True)
            preds = mlp(x_train[batch_idx])
            loss = criterion(preds, y_train[batch_idx])
            loss.backward()
            optimizer.step()

        mlp.eval()
        with torch.no_grad():
            preds = mlp(x_test)
            mse = criterion(preds, y_test).item()
        test_history.append(mse)
        best_mse = min(best_mse, mse)

    final_mse = test_history[-1] if test_history else float("inf")
    return mlp, best_mse, final_mse, test_history


def _train_regression_probe_with_model_from_splits(
    train_features: torch.Tensor,
    train_targets: torch.Tensor,
    test_features: torch.Tensor,
    test_targets: torch.Tensor,
    epochs: int,
    lr: float,
    hidden_dim: int,
    batch_size: int,
    device: str | None,
    seed: int | None,
) -> tuple[SimpleMLP, float, float, list[float]]:
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))

    mlp = SimpleMLP(train_features.shape[-1], hidden_dim, train_targets.shape[-1]).to(device)
    optimizer = torch.optim.Adam(mlp.parameters(), lr=lr)
    criterion = nn.MSELoss()

    test_history: list[float] = []
    best_mse = float("inf")
    for _ in range(epochs):
        mlp.train()
        perm_train = torch.randperm(train_features.shape[0], device=device)
        for start in range(0, train_features.shape[0], batch_size):
            batch_idx = perm_train[start : start + batch_size]
            optimizer.zero_grad(set_to_none=True)
            preds = mlp(train_features[batch_idx])
            loss = criterion(preds, train_targets[batch_idx])
            loss.backward()
            optimizer.step()

        mlp.eval()
        with torch.no_grad():
            preds = mlp(test_features)
            mse = criterion(preds, test_targets).item()
        test_history.append(mse)
        best_mse = min(best_mse, mse)

    final_mse = test_history[-1] if test_history else float("inf")
    return mlp, best_mse, final_mse, test_history


def train_mlp_autoencoder_on_reconstruction(
    inputs: torch.Tensor,
    epochs: int = 100,
    lr: float = 1e-3,
    hidden_dims: tuple[int, ...] = (512, 256),
    train_split: float = 0.8,
    batch_size: int = 64,
    device: str | None = None,
    seed: int | None = None,
    accuracy_threshold: float = 0.1,
) -> tuple[float, float, list[float]]:
    """Train a simple MLP autoencoder and report reconstruction accuracy.

    Accuracy is the fraction of pixels with absolute error below accuracy_threshold.
    Returns (best_test_accuracy, final_test_accuracy, test_accuracy_history).
    """
    if seed is not None:
        torch.manual_seed(seed)

    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    inputs = inputs.to(device)
    input_dim = inputs[0].numel()
    model = MLPImageAutoencoder(input_dim, hidden_dims=hidden_dims).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()

    num_samples = inputs.shape[0]
    num_train = int(num_samples * train_split)
    perm = torch.randperm(num_samples, device=device)
    train_idx = perm[:num_train]
    test_idx = perm[num_train:]

    train_data = inputs[train_idx]
    test_data = inputs[test_idx]

    test_history: list[float] = []
    best_accuracy = 0.0
    for _ in range(epochs):
        model.train()
        perm_train = torch.randperm(train_data.shape[0], device=device)
        for start in range(0, train_data.shape[0], batch_size):
            batch_idx = perm_train[start : start + batch_size]
            batch = train_data[batch_idx]
            optimizer.zero_grad(set_to_none=True)
            recon = model(batch)
            loss = criterion(recon, batch)
            loss.backward()
            optimizer.step()

        model.eval()
        with torch.no_grad():
            recon = model(test_data)
            accuracy = (recon.sub(test_data).abs() < accuracy_threshold).float().mean().item()
        test_history.append(accuracy)
        best_accuracy = max(best_accuracy, accuracy)

    final_accuracy = test_history[-1] if test_history else 0.0
    return best_accuracy, final_accuracy, test_history


if __name__ == "__main__":
    # Example usage: place images in data/train/class_name/*.jpg
    trained_model, loss_history = run_training("data/train")
    print("Final loss:", loss_history[-1])

    # Inspect the inputs/outputs/weights from the first encoder attention layer.
    sample = torch.randn(
        1,
        trained_model.config.in_channels,
        trained_model.config.image_size,
        trained_model.config.image_size,
    )
    reconstructed, first_attn = trained_model(sample, return_first_attn=True)
    if first_attn is not None:
        print("First attention input shape:", first_attn.inputs.shape)
        print("First attention output shape:", first_attn.outputs.shape)
        print("First attention weight shape:", first_attn.weights.shape)

    # Lightweight demo to print the actual first attention inputs/outputs.
    torch.manual_seed(0)
    demo_config = ViTAEConfig(
        image_size=8,
        patch_size=4,
        in_channels=3,
        embed_dim=16,
        encoder_depth=1,
        decoder_depth=1,
        num_heads=4,
        dropout=0.0,
    )
    demo_model = ViTImageAutoencoder(demo_config)
    demo_sample = torch.randn(
        1,
        demo_config.in_channels,
        demo_config.image_size,
        demo_config.image_size,
    )
    demo_attn = demo_model.get_first_attention_io(demo_sample)
    print("Demo first attention input:", demo_attn.inputs)
    print("Demo first attention output:", demo_attn.outputs)
