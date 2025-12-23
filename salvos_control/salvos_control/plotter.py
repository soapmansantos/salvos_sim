#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import pygame

np.set_printoptions(legacy='1.25')


class RealtimePlotter:
    """
    Very simple real-time plotter for pygame.
    Input each tick: dict like {'p': 0.12, 'q': -0.03, 'r': 0.4}
    - One stacked panel per key (order = first-seen).
    - Autoscale per panel with small padding.
    - Keeps a fixed-length history (scrolls left).
    """

    def __init__(self,
                 history_len=600,
                 px_per_panel=120,
                 margin_px=6,
                 grid_x=4,
                 grid_y=3,
                 auto_range=True,
                 range_pad=0.1,
                 font_name=None):
        self.history_len = int(history_len)
        self.px_per_panel = int(px_per_panel)
        self.margin_px = int(margin_px)
        self.grid_x = int(grid_x)
        self.grid_y = int(grid_y)
        self.auto_range = bool(auto_range)
        self.range_pad = float(range_pad)
        self.font_name = font_name

        self.keys = []          # order of panels
        self.buf = {}           # key -> np.array(history_len) of floats (nan = empty)
        self.font = None        # lazy init

        # deterministic simple palette
        self.palette = [
            (235, 94, 52), (52, 136, 235), (46, 204, 113), (241, 196, 15),
            (155, 89, 182), (26, 188, 156), (231, 76, 60), (52, 73, 94)
        ]
        self.color = {}

    def _ensure_key(self, key):
        if key in self.buf:
            return
        self.buf[key] = np.full(self.history_len, np.nan, dtype=float)
        self.keys.append(key)
        self.color[key] = self.palette[(len(self.keys)-1) % len(self.palette)]

    def push(self, samples: dict[str, float]):
        """Append one sample per provided key (others hold last)."""
        for k, v in samples.items():
            self._ensure_key(k)
            # scroll left, write new at end
            self.buf[k][:-1] = self.buf[k][1:]
            self.buf[k][-1] = float(v)

        # also scroll keys that didn’t get a new value (keep continuity)
        for k in self.keys:
            if k not in samples:
                self.buf[k][:-1] = self.buf[k][1:]
                self.buf[k][-1] = self.buf[k][-2]  # hold last

    def _init_font(self):
        if self.font is None:
            try:
                self.font = pygame.font.SysFont(self.font_name, 14)
            except Exception:
                self.font = pygame.font.Font(None, 14)

    def draw(self, screen: pygame.Surface):
        """Draw stacked panels; call once per frame after push()."""
        self._init_font()
        w, h = screen.get_size()
        n = max(1, len(self.keys))
        panel_h = min(self.px_per_panel, max(80, h // n))
        usable_h = panel_h * n
        top0 = (h - usable_h) // 2

        # background block
        pygame.draw.rect(screen, (16, 16, 18), pygame.Rect(0, top0, w, usable_h))

        for i, k in enumerate(self.keys):
            rect = pygame.Rect(self.margin_px,
                               top0 + i*panel_h + self.margin_px,
                               w - 2*self.margin_px,
                               panel_h - 2*self.margin_px)
            self._draw_panel(screen, rect, k)

    def _draw_panel(self, screen, rect, key):
        # panel bg
        pygame.draw.rect(screen, (24, 24, 26), rect, border_radius=6)
        pygame.draw.rect(screen, (46, 46, 52), rect, 1, border_radius=6)

        # grid
        self._draw_grid(screen, rect)

        # data and scaling
        y = self.buf[key]
        col = self.color[key]
        finite = np.isfinite(y)
        if not finite.any():
            return

        yv = y[finite]
        ymin, ymax = float(np.min(yv)), float(np.max(yv))
        if self.auto_range:
            if math.isclose(ymin, ymax, abs_tol=1e-6):
                pad = 1.0 if abs(ymin) < 1e-3 else 0.1*abs(ymin)
                ymin, ymax = ymin - pad, ymax + pad
            else:
                pad = self.range_pad * (ymax - ymin)
                ymin, ymax = ymin - pad, ymax + pad
        else:
            if ymin == ymax:
                ymin, ymax = ymin - 1.0, ymax + 1.0

        # map to pixels
        xs = np.linspace(rect.left, rect.right-1, self.history_len)
        # normalize y -> [0,1]
        denom = max(1e-9, (ymax - ymin))
        ky = (y - ymin) / denom
        ky = np.clip(ky, 0.0, 1.0)
        ys = rect.bottom - 1 - ky * (rect.height - 1)

        # build polyline (skip NaNs)
        pts = [(int(x), int(ys[i])) for i, x in enumerate(xs) if np.isfinite(y[i])]
        if len(pts) >= 2:
            pygame.draw.lines(screen, col, False, pts, 2)

        # zero line (if within range)
        if ymin < 0 < ymax:
            z = rect.bottom - 1 - (-ymin/denom) * (rect.height - 1)
            pygame.draw.line(screen, (60, 60, 68), (rect.left, int(z)), (rect.right, int(z)), 1)

        # label
        name = self.font.render(f'{key}', True, col)
        screen.blit(name, (rect.left + 6, rect.top + 4))
        min_s = self.font.render(f'{ymin:.3g}', True, (140, 140, 148))
        max_s = self.font.render(f'{ymax:.3g}', True, (140, 140, 148))
        screen.blit(min_s, (rect.right - 64, rect.bottom - 18))
        screen.blit(max_s, (rect.right - 64, rect.top + 4))

    def _draw_grid(self, screen, rect):
        # vertical
        if self.grid_x > 0:
            step = rect.width / self.grid_x
            for i in range(1, self.grid_x):
                x = rect.left + int(i * step)
                pygame.draw.line(screen, (36, 36, 42), (x, rect.top), (x, rect.bottom), 1)
        # horizontal
        if self.grid_y > 0:
            step = rect.height / self.grid_y
            for i in range(1, self.grid_y):
                y = rect.top + int(i * step)
                pygame.draw.line(screen, (36, 36, 42), (rect.left, y), (rect.right, y), 1)
