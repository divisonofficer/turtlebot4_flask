import torch
import torch.nn as nn
import torch.nn.functional as F


class ResidualBlock(nn.Module):
    def __init__(self, in_planes, planes, norm_fn="group", stride=1):
        super(ResidualBlock, self).__init__()

        self.conv1 = nn.Conv2d(
            in_planes, planes, kernel_size=3, padding=1, stride=stride
        )
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, padding=1)
        self.relu = nn.ReLU(inplace=True)

        num_groups = planes // 8

        if norm_fn == "group":
            self.norm1 = nn.GroupNorm(num_groups=num_groups, num_channels=planes)
            self.norm2 = nn.GroupNorm(num_groups=num_groups, num_channels=planes)
            if not (stride == 1 and in_planes == planes):
                self.norm3 = nn.GroupNorm(num_groups=num_groups, num_channels=planes)

        elif norm_fn == "batch":
            self.norm1 = nn.BatchNorm2d(planes)
            self.norm2 = nn.BatchNorm2d(planes)
            if not (stride == 1 and in_planes == planes):
                self.norm3 = nn.BatchNorm2d(planes)

        elif norm_fn == "instance":
            self.norm1 = nn.InstanceNorm2d(planes)
            self.norm2 = nn.InstanceNorm2d(planes)
            if not (stride == 1 and in_planes == planes):
                self.norm3 = nn.InstanceNorm2d(planes)

        elif norm_fn == "none":
            self.norm1 = nn.Sequential()
            self.norm2 = nn.Sequential()
            if not (stride == 1 and in_planes == planes):
                self.norm3 = nn.Sequential()

        if stride == 1 and in_planes == planes:
            self.downsample = None

        else:
            self.downsample = nn.Sequential(
                nn.Conv2d(in_planes, planes, kernel_size=1, stride=stride), self.norm3
            )

    def forward(self, x):
        y = x
        y = self.conv1(y)
        y = self.norm1(y)
        y = self.relu(y)
        y = self.conv2(y)
        y = self.norm2(y)
        y = self.relu(y)

        if self.downsample is not None:
            x = self.downsample(x)

        return self.relu(x + y)


class BottleneckBlock(nn.Module):
    def __init__(self, in_planes, planes, norm_fn="group", stride=1):
        super(BottleneckBlock, self).__init__()

        self.conv1 = nn.Conv2d(in_planes, planes // 4, kernel_size=1, padding=0)
        self.conv2 = nn.Conv2d(
            planes // 4, planes // 4, kernel_size=3, padding=1, stride=stride
        )
        self.conv3 = nn.Conv2d(planes // 4, planes, kernel_size=1, padding=0)
        self.relu = nn.ReLU(inplace=True)

        num_groups = planes // 8

        if norm_fn == "group":
            self.norm1 = nn.GroupNorm(num_groups=num_groups, num_channels=planes // 4)
            self.norm2 = nn.GroupNorm(num_groups=num_groups, num_channels=planes // 4)
            self.norm3 = nn.GroupNorm(num_groups=num_groups, num_channels=planes)
            if not stride == 1:
                self.norm4 = nn.GroupNorm(num_groups=num_groups, num_channels=planes)

        elif norm_fn == "batch":
            self.norm1 = nn.BatchNorm2d(planes // 4)
            self.norm2 = nn.BatchNorm2d(planes // 4)
            self.norm3 = nn.BatchNorm2d(planes)
            if not stride == 1:
                self.norm4 = nn.BatchNorm2d(planes)

        elif norm_fn == "instance":
            self.norm1 = nn.InstanceNorm2d(planes // 4)
            self.norm2 = nn.InstanceNorm2d(planes // 4)
            self.norm3 = nn.InstanceNorm2d(planes)
            if not stride == 1:
                self.norm4 = nn.InstanceNorm2d(planes)

        elif norm_fn == "none":
            self.norm1 = nn.Sequential()
            self.norm2 = nn.Sequential()
            self.norm3 = nn.Sequential()
            if not stride == 1:
                self.norm4 = nn.Sequential()

        if stride == 1:
            self.downsample = None

        else:
            self.downsample = nn.Sequential(
                nn.Conv2d(in_planes, planes, kernel_size=1, stride=stride), self.norm4
            )

    def forward(self, x):
        y = x
        y = self.relu(self.norm1(self.conv1(y)))
        y = self.relu(self.norm2(self.conv2(y)))
        y = self.relu(self.norm3(self.conv3(y)))

        if self.downsample is not None:
            x = self.downsample(x)

        return self.relu(x + y)


class BasicEncoder(nn.Module):
    def __init__(
        self, output_dim=128, norm_fn="batch", input_dim=3, dropout=0.0, downsample=3
    ):
        super(BasicEncoder, self).__init__()
        self.norm_fn = norm_fn
        self.downsample = downsample
        self.input_dim = input_dim

        if self.norm_fn == "group":
            self.norm1 = nn.GroupNorm(num_groups=8, num_channels=64)

        elif self.norm_fn == "batch":
            self.norm1 = nn.BatchNorm2d(64)

        elif self.norm_fn == "instance":
            self.norm1 = nn.InstanceNorm2d(64)

        elif self.norm_fn == "none":
            self.norm1 = nn.Sequential()

        self.conv1 = nn.Conv2d(
            input_dim, 64, kernel_size=7, stride=1 + (downsample > 2), padding=3
        )
        self.relu1 = nn.ReLU(inplace=True)

        self.in_planes = 64
        self.layer1 = self._make_layer(64, stride=1)
        self.layer2 = self._make_layer(96, stride=1 + (downsample > 1))
        self.layer3 = self._make_layer(128, stride=1 + (downsample > 0))

        # output convolution
        self.conv2 = nn.Conv2d(128, output_dim, kernel_size=1)

        self.dropout = None
        if dropout > 0:
            self.dropout = nn.Dropout2d(p=dropout)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
            elif isinstance(m, (nn.BatchNorm2d, nn.InstanceNorm2d, nn.GroupNorm)):
                if m.weight is not None:
                    nn.init.constant_(m.weight, 1)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def _make_layer(self, dim, stride=1):
        layer1 = ResidualBlock(self.in_planes, dim, self.norm_fn, stride=stride)
        layer2 = ResidualBlock(dim, dim, self.norm_fn, stride=1)
        layers = (layer1, layer2)

        self.in_planes = dim
        return nn.Sequential(*layers)

    def forward(self, x, dual_inp=False):

        # if input is list, combine batch dimension
        is_list = isinstance(x, tuple) or isinstance(x, list)
        if is_list:
            batch_dim = x[0].shape[0]
            x = torch.cat(x, dim=0)

        x = self.conv1(x)
        x = self.norm1(x)
        x = self.relu1(x)

        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)

        x = self.conv2(x)

        if self.training and self.dropout is not None:
            x = self.dropout(x)

        if is_list:
            x = x.split(split_size=batch_dim, dim=0)

        return x


class LocalAttentionModule(nn.Module):
    def __init__(self, in_channels, reduction=16):
        super(LocalAttentionModule, self).__init__()
        self.local_conv1 = nn.Conv2d(
            in_channels, in_channels // reduction, kernel_size=1
        )
        self.local_bn1 = nn.BatchNorm2d(
            in_channels // reduction, track_running_stats=False
        )
        self.local_relu = nn.ReLU(inplace=False)
        self.local_conv2 = nn.Conv2d(
            in_channels // reduction, in_channels, kernel_size=1
        )
        self.local_bn2 = nn.BatchNorm2d(in_channels, track_running_stats=False)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
            elif isinstance(
                m, (nn.BatchNorm2d, nn.InstanceNorm2d, nn.GroupNorm, nn.SyncBatchNorm)
            ):
                if m.weight is not None:
                    nn.init.constant_(m.weight, 1)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def forward(self, x):
        local_branch = self.local_conv1(x)
        local_branch = self.local_bn1(local_branch)
        local_branch = self.local_relu(local_branch)
        local_branch = self.local_conv2(local_branch)
        local_branch = self.local_bn2(local_branch)
        return local_branch


class GlobalAttentionModule(nn.Module):
    def __init__(self, in_channels, reduction=16):
        super(GlobalAttentionModule, self).__init__()
        self.in_channels = in_channels
        self.reduction = reduction

        # Global average pooling branch
        self.global_avg_pool = nn.AdaptiveAvgPool2d(1)

        # First branch
        self.global_conv1 = nn.Conv2d(
            in_channels, in_channels // reduction, kernel_size=1
        )
        self.global_bn1 = nn.BatchNorm2d(
            in_channels // reduction, track_running_stats=False, eps=0.001
        )
        self.global_relu = nn.ReLU(inplace=False)
        self.global_conv2 = nn.Conv2d(
            in_channels // reduction, in_channels, kernel_size=1
        )
        self.global_bn2 = nn.BatchNorm2d(
            in_channels, track_running_stats=False, eps=0.001
        )

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.xavier_normal_(m.weight)
                m.weight.data *= 0.1
            elif isinstance(
                m, (nn.BatchNorm2d, nn.InstanceNorm2d, nn.GroupNorm, nn.SyncBatchNorm)
            ):
                if m.weight is not None:
                    nn.init.constant_(m.weight, 1)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def forward(self, x):
        # Global average pooling branch
        avg_pool = self.global_avg_pool(x).float()

        # First branch

        global_branch = self.global_conv1(avg_pool)
        global_branch = self.global_bn1(global_branch)

        global_branch = self.global_relu(global_branch)

        global_branch = self.global_conv2(global_branch)

        global_branch = self.global_bn2(global_branch)
        return global_branch


class MultiScaleChannelAttentionModule(nn.Module):
    def __init__(self, in_channels, reduction=16):
        super(MultiScaleChannelAttentionModule, self).__init__()
        self.in_channels = in_channels
        self.reduction = reduction

        self.local_attention = LocalAttentionModule(in_channels, reduction)
        self.global_attention = GlobalAttentionModule(in_channels, reduction)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        out = self.local_attention(x) + self.global_attention(x)

        out = self.sigmoid(out)

        return out


class AttentionFeatureFusion(nn.Module):
    def __init__(self, in_channels=128, reduction=4):
        super(AttentionFeatureFusion, self).__init__()

        self.attention_rgb = MultiScaleChannelAttentionModule(in_channels, reduction)
        self.attention_nir = MultiScaleChannelAttentionModule(in_channels, reduction)

        self.attention_fusion = MultiScaleChannelAttentionModule(in_channels, reduction)

    def forward(self, rgb, nir, debug_attention=False):
        # Apply the attention modules to the input features
        rgb_att = self.attention_rgb(rgb)
        nir_att = self.attention_nir(nir)

        # Concatenate the attention features
        sum_att = rgb_att + nir_att + 1e-6
        rgb_att = rgb * rgb_att / sum_att * 2
        nir_att = nir * nir_att / sum_att * 2
        att_features = rgb_att + nir_att

        # Apply the attention fusion module
        att_fusion = self.attention_fusion(att_features)

        out = att_fusion * rgb_att + (1 - att_fusion) * nir_att
        if debug_attention:
            return att_fusion, rgb, nir
        return out


class GuidedFilter(nn.Module):
    def __init__(self):
        super(GuidedFilter, self).__init__()

    def forward(self, I, p, radius=3, eps=1e-6):
        """
        Perform guided filtering on input images using PyTorch.

        Args:
            I (torch.Tensor): Guide image tensor of shape (batch_size, 1, height, width).
            p (torch.Tensor): Input image tensor to be filtered of shape (batch_size, 3, height, width).
            radius (int, optional): Radius of the window. Default is 15.
            eps (float, optional): Regularization parameter to avoid division by zero. Default is 1e-6.

        Returns:
            torch.Tensor: Filtered image tensor of shape (batch_size, 3, height, width).
        """
        # Ensure the guide image has a single channel
        assert (
            I.dim() == 4 and I.size(1) == 1
        ), "Guide image I must have shape (batch, 1, H, W)"
        # Ensure the input image has three channels
        assert (
            p.dim() == 4 and p.size(1) == 3
        ), "Input image p must have shape (batch, 3, H, W)"

        batch_size, _, height, width = I.size()
        window_size = (2 * radius + 1) ** 2

        # Define a box filter kernel
        # The kernel has shape (channels, 1, kernel_size, kernel_size) and is normalized
        # to compute the mean.
        def box_filter(x):
            # x: (batch, channels, height, width)
            channels = x.size(1)
            kernel = (
                torch.ones(
                    (channels, 1, 2 * radius + 1, 2 * radius + 1),
                    dtype=x.dtype,
                    device=x.device,
                )
                / window_size
            )
            return F.conv2d(
                x, weight=kernel, bias=None, stride=1, padding=radius, groups=channels
            )

        # Step 1: Compute mean of I, p, I*p, and I*I
        mean_I = box_filter(I)  # (batch, 1, H, W)
        mean_p = box_filter(p)  # (batch, 3, H, W)
        mean_Ip = box_filter(I * p)  # (batch, 3, H, W)
        mean_II = box_filter(I * I)  # (batch, 1, H, W)

        # Step 2: Compute covariance of (I, p) and variance of I
        cov_Ip = mean_Ip - mean_I * mean_p  # (batch, 3, H, W)
        var_I = mean_II - mean_I * mean_I  # (batch, 1, H, W)

        # Step 3: Compute a and b
        a = cov_Ip / (var_I + eps)  # (batch, 3, H, W)
        b = mean_p - a * mean_I  # (batch, 3, H, W)

        # Step 4: Compute mean of a and b
        mean_a = box_filter(a)  # (batch, 3, H, W)
        mean_b = box_filter(b)  # (batch, 3, H, W)

        # Step 5: Compute the output image
        q = mean_a * I + mean_b  # Broadcasting I from (batch,1,H,W) to (batch,3,H,W)

        # Optionally, clamp the output to valid image range
        q = torch.clamp(q, 0, 1)

        return q


class RGBHSV(nn.Module):
    def __init__(self):
        super(RGBHSV, self).__init__()

    def forward(self, rgb: torch.Tensor):
        """
        RGB 텐서를 HSV 텐서로 변환합니다.

        입력:
            rgb: Tensor of shape (b, 3, h, w) with values in [0, 1]

        출력:
            hsv: Tensor of shape (b, 3, h, w) with H in [0, 360], S and V in [0, 1]
        """
        r, g, b = rgb[:, 0, :, :], rgb[:, 1, :, :], rgb[:, 2, :, :]

        maxc, _ = rgb.max(dim=1)
        minc, _ = rgb.min(dim=1)
        delta = maxc - minc + 1e-10  # 작은 값을 더해 division by zero 방지

        # Hue 계산
        mask = delta > 0
        h = torch.zeros_like(maxc)

        # Red is max
        mask_r = (maxc == r) & mask
        h[mask_r] = (60 * ((g[mask_r] - b[mask_r]) / delta[mask_r])) % 360

        # Green is max
        mask_g = (maxc == g) & mask
        h[mask_g] = (60 * ((b[mask_g] - r[mask_g]) / delta[mask_g]) + 120) % 360

        # Blue is max
        mask_b = (maxc == b) & mask
        h[mask_b] = (60 * ((r[mask_b] - g[mask_b]) / delta[mask_b]) + 240) % 360

        # Saturation 계산
        s = torch.zeros_like(maxc)
        s[maxc != 0] = delta[maxc != 0] / maxc[maxc != 0]

        # Value 계산
        v = maxc

        hsv = torch.stack([h, s, v], dim=1)
        return hsv


class HSVRGB(nn.Module):
    def __init__(self):
        super(HSVRGB, self).__init__()

    def forward(self, hsv):
        """
        HSV 텐서를 RGB 텐서로 변환합니다.

        입력:
            hsv: Tensor of shape (b, 3, h, w)
                - H in [0, 360]
                - S in [0, 1]
                - V in [0, 1]

        출력:
            rgb: Tensor of shape (b, 3, h, w) with values in [0, 1]
        """
        h, s, v = hsv[:, 0, :, :], hsv[:, 1, :, :], hsv[:, 2, :, :]

        c = v * s  # 채도와 명도를 이용해 채도
        h_prime = h / 60.0  # Hue를 60으로 나눠 섹션 결정
        x = c * (1 - torch.abs((h_prime % 2) - 1))

        zero = torch.zeros_like(h)

        # 각 섹션에 따른 RGB 중간값 계산
        cond = (h_prime >= 0) & (h_prime < 1)
        r = torch.where(cond, c, torch.zeros_like(c))
        g = torch.where(cond, x, torch.zeros_like(x))
        b = torch.zeros_like(x)

        cond = (h_prime >= 1) & (h_prime < 2)
        r = torch.where(cond, x, r)
        g = torch.where(cond, c, g)

        cond = (h_prime >= 2) & (h_prime < 3)
        g = torch.where(cond, c, g)
        b = torch.where(cond, x, b)

        cond = (h_prime >= 3) & (h_prime < 4)
        g = torch.where(cond, x, g)
        b = torch.where(cond, c, b)

        cond = (h_prime >= 4) & (h_prime < 5)
        r = torch.where(cond, x, r)
        b = torch.where(cond, c, b)

        cond = (h_prime >= 5) & (h_prime < 6)
        r = torch.where(cond, c, r)
        b = torch.where(cond, x, b)

        m = v - c
        r = r + m
        g = g + m
        b = b + m

        rgb = torch.stack([r, g, b], dim=1)
        return rgb


class HSVNet(torch.nn.Module):

    def __init__(self, args):
        super(HSVNet, self).__init__()
        self.args = args
        self.encoder = BasicEncoder(downsample=2, output_dim=256)
        self.fusion = AttentionFeatureFusion(in_channels=256, reduction=4)
        self.channel_reduction = nn.Sequential(
            ResidualBlock(256, 128),
            ResidualBlock(128, 64),
            nn.Conv2d(64, 2, 3, padding=1),
            nn.ReLU(),
        )
        self.hsv2rgb = HSVRGB()
        self.rgb2hsv = RGBHSV()
        self.gf = GuidedFilter()
        self.padder = None

    def pad(self, img, div=32):
        B, C, H, W = img.size()
        pad_h = 0 if H % div == 0 else div - (H % div)
        pad_w = 0 if W % div == 0 else div - (W % div)
        self.padder = nn.ZeroPad2d((0, pad_w, 0, pad_h))
        return self.padder(img)

    def forward(
        self,
        v: list[torch.Tensor],
        n: list[torch.Tensor],
    ):

        islist = isinstance(v, list)
        if islist:
            v = torch.cat(v, dim=0)
            n = torch.cat(n, dim=0)
        B, _, H, W = v.size()

        v = self.pad(v)
        n = self.pad(n)

        hsv = self.rgb2hsv(v)

        hsv_input = (v / 255 * 2 - 1).contiguous()
        n_input = (n / 255 * 2 - 1).contiguous()

        hsv_fmap = self.encoder(hsv_input)
        nir_fmap = self.encoder(n_input.repeat(1, 3, 1, 1))
        fusion_fmap = self.fusion(hsv_fmap, nir_fmap)

        w = F.sigmoid(self.channel_reduction(fusion_fmap))

        w = F.interpolate(w, scale_factor=4, mode="bilinear", align_corners=False)

        rgb = self.hsv2rgb(
            torch.concat(
                [hsv[:, :1], hsv[:, 1:2], w[:, :1] * hsv[:, 2:3] + w[:, 1:] * n],
                dim=1,
            )
        )
        # rgb = self.gf(n, rgb, radius=5) * 255

        rgb = torch.clip(rgb[..., :H, :W], 0, 255)

        return rgb

    def freeze_bn(self):
        for m in self.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.requires_grad_(False)
                m.eval()

    def freeze_raft(
        self,
    ):
        for name, param in self.encoder.named_parameters():
            param.requires_grad_(False)
        for name, param in self.raft_stereo.named_parameters():
            param.requires_grad_(False)
        self.freeze_bn()
